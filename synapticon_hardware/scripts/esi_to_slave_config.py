#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""
Generate the slave_config of a Synapticon SOMANET drive from the ESI file of its firmware.

    esi_to_slave_config.py --esi SOMANET_CiA_402.xml --product 0x302 --out my_drive.yaml

Synapticon ships an ESI file with every firmware release, and the product code and revision are listed in it.
Without --revision, the first device with the product code is taken.

| Variant      | Modes             | Impedance gains and torque limit | Starts in |
|--------------|-------------------|----------------------------------|-----------|
| default      | CSP and impedance | mapped                           | -6        |
| --csp-only   | CSP               | left out                         | 8         |

With the default, the robot description picks the mode of each joint. Every variant adds a diagnostics TxPDO,
about 28 bytes per drive and cycle.

PDO_LAYOUT and STARTUP_SDOS below decide which objects are exchanged and which ros2_control interface each one
serves. From the ESI file come the device's identity, its distributed clock setting and the data types. When the
device cannot map an object of the layout, nothing is written.
"""

import argparse
from dataclasses import dataclass, field
import math
import os
import sys
import xml.etree.ElementTree as ET

# Variant names. A layout entry or a startup SDO belongs to one or both of them.
IMPEDANCE = "impedance"
CSP = "csp"
BOTH = frozenset({IMPEDANCE, CSP})
IMPEDANCE_ONLY = frozenset({IMPEDANCE})
CSP_ONLY = frozenset({CSP})

# Direction of a PDO, also the prefix of the interface key of its entries.
COMMAND = "command"
STATE = "state"

NAN = float("nan")

# The layout and SDO tables come first, since they are what a new drive or a new interface changes.


@dataclass(frozen=True)
class PdoEntry:
    """One object in a PDO.

    `interface` is the ros2_control interface it serves, None maps it without using it. `scaled` entries carry a
    factor of 1, since the HAL converts to SI units itself. `default` is what a command entry sends while its
    command is unset. NaN sets no default.
    """

    index: int
    sub_index: int
    note: str
    interface: str | None = None
    scaled: bool = False
    default: int | float | None = None
    variants: frozenset[str] = BOTH


@dataclass(frozen=True)
class Pdo:
    """A PDO and its entries in mapping order. COMMAND PDOs go from the master to the drive, STATE PDOs back."""

    index: int
    direction: str
    entries: tuple[PdoEntry, ...]


@dataclass(frozen=True)
class StartupSdo:
    """
    An SDO written in PRE-OP at every bring-up. An inactive one is written as a comment to enable by hand.

    An SI one goes into the `sdo_si` list: its value is in SI units, and bring-up converts it into the drive's unit.
    """

    index: int
    sub_index: int
    type: str
    value: int | float
    note: str
    active: bool = True
    variants: frozenset[str] = BOTH
    si: bool = False


# fmt: off
PDO_LAYOUT = (
    Pdo(0x1600, COMMAND, (
        PdoEntry(0x6040, 0, "controlword, set by the CiA 402 state handling only", "control_word", default=0),
        PdoEntry(0x6060, 0, "mode of operation, -6 (impedance) while no mode is commanded", "mode_of_operation",
                 default=-6, variants=IMPEDANCE_ONLY),
        PdoEntry(0x6060, 0, "mode of operation, 8 (CSP) while no mode is commanded", "mode_of_operation",
                 default=8, variants=CSP_ONLY),
        PdoEntry(0x607A, 0, "rest position of the impedance spring, or the CSP setpoint", "position",
                 scaled=True, default=NAN, variants=IMPEDANCE_ONLY),
        PdoEntry(0x607A, 0, "position setpoint", "position", scaled=True, default=NAN, variants=CSP_ONLY),
        PdoEntry(0x60FF, 0, "velocity setpoint or feedforward", "velocity", scaled=True, default=0),
        PdoEntry(0x2053, 0, "extra output torque [mNm], fed by the effort command", "joint_torque_offset",
                 scaled=True, default=0, variants=IMPEDANCE_ONLY),
        PdoEntry(0x2060, 1, "impedance stiffness [mNm/rad], sent every cycle", "stiffness",
                 scaled=True, default=0, variants=IMPEDANCE_ONLY),
        PdoEntry(0x2060, 2, "impedance damping, sent every cycle", "damping",
                 scaled=True, default=0, variants=IMPEDANCE_ONLY),
        PdoEntry(0x2054, 0, "torque cap [mNm], sent every cycle. Its first value is the joint's max_joint_torque "
                 "if set, else what the drive has stored", "max_joint_torque",
                 scaled=True, default=0, variants=IMPEDANCE_ONLY),
    )),
    Pdo(0x1A00, STATE, (
        PdoEntry(0x6041, 0, "statusword with the CiA 402 state", "status_word"),
        PdoEntry(0x6061, 0, "mode the drive is running in", "mode_of_operation"),
        PdoEntry(0x6064, 0, "measured position", "position", scaled=True),
        PdoEntry(0x606C, 0, "measured velocity", "velocity", scaled=True),
        PdoEntry(0x6077, 0, "measured torque", "effort", scaled=True),
        PdoEntry(0x603F, 0, "code of the last error", "error_code"),
    )),
    # Diagnostics. These objects are in no predefined PDO, they come from the object dictionary.
    Pdo(0x1A02, STATE, (
        PdoEntry(0x6079, 0, "DC link voltage [mV -> V]", "dc_link_voltage", scaled=True),
        PdoEntry(0x2031, 1, "inverter temperature [millidegree C -> degree C], unit assumed",
                 "temperature_inverter", scaled=True),
        PdoEntry(0x2030, 1, "core board temperature [millidegree C -> degree C], unit assumed",
                 "temperature_core", scaled=True),
        PdoEntry(0x2041, 1, "motor current, magnitude of (Id, Iq) [mA -> A]",
                 "motor_current", scaled=True),
        PdoEntry(0x203F, 1, "error report: 8 ASCII bytes in a real64 slot, decoded by the HAL and the drive "
                 "controller", "error_report"),
        PdoEntry(0x2052, 0, "torque demand after the cap [mNm -> Nm]", "joint_torque_demand",
                 scaled=True),
    )),
)

STARTUP_SDOS = (
    StartupSdo(0x605A, 0, "int16", 6, "quick stop option code"),
    StartupSdo(0x6085, 0, "uint32", 200.0, "quick stop deceleration in rad/s^2 at the motor, see the note on 0x605A",
               active=False, si=True),
    StartupSdo(0x2060, 3, "real32", 1.0, "gain of the inner torque loop", variants=IMPEDANCE_ONLY),
    StartupSdo(0x2150, 1, "uint8", 1, "torque sensor type, see the impedance note", active=False,
               variants=IMPEDANCE_ONLY),
)
# fmt: on

# Header comments of the generated file, per topic. The variants pick the topics they need.
NOTES = {
    "units": """\
All factors are 1. The HAL reads the units of the drive at bring-up and converts to SI units itself.
""",
    "modes": """\
The robot description picks the mode of each joint: mode_of_operation 8 is CSP, -6 is impedance.
""",
    "startup": """\
Startup SDOs are written in PRE-OP at every bring-up. Check each value for your drive and your machine.

0x605A, the quick stop option code, decides what a quick_stop request does:
  6  ramps down and stays powered in Quick Stop Active, holding the joint
  2  ramps down and then takes the power off (the factory setting)
  any other value disables the drive at once
Bring-up reads 0x605A back, and the quick stop deceleration 0x6085 too. This file leaves 0x6085 to the
commissioning of the drive. To set it at every bring-up, enable its sdo_si entry below.

'sdo' holds values without a unit, written as they are. An object whose unit depends on the drive's
commissioning goes into 'sdo_si' with si_value in SI units at the motor: bring-up reads the drive's
units first and converts it. Any other object in 'sdo' needs 'raw: true' to say the value is meant as written.
""",
    "impedance": """\
Impedance mode:
  0x2060:3  gain of the inner torque loop. Raise it in small steps.
  0x2150:1  torque sensor type, must be set. With no sensor: 1 (analog input), all analog coefficients 0.
            Store it on the drive once, or enable the 0x2150 entry below to write it at every bring-up.
The stiffness, the damping and the torque ceiling (0x2054) are streamed. They come from the commands, and
from the robot definition where a command leaves them out.
""",
}


@dataclass(frozen=True)
class Variant:
    """One kind of slave_config this script writes."""

    name: str
    title: str
    notes: tuple[str, ...]


VARIANTS = {
    IMPEDANCE: Variant(
        IMPEDANCE,
        "for CSP and impedance mode",
        ("units", "modes", "startup", "impedance"),
    ),
    CSP: Variant(CSP, "for CSP", ("units", "startup")),
}

# The distributed clock activation when the ESI file names none.
DEFAULT_ASSIGN_ACTIVATE = 0x0300

# Slave_config type of each ESI base type.
SLAVE_TYPES = {
    "BOOL": "bool",
    "SINT": "int8",
    "USINT": "uint8",
    "INT": "int16",
    "UINT": "uint16",
    "DINT": "int32",
    "UDINT": "uint32",
    "LINT": "int64",
    "ULINT": "uint64",
    "REAL": "real32",
    "LREAL": "real64",
}

# Fallback for a vendor type: a signed integer of the same width.
SIGNED_TYPE_OF_WIDTH = {8: "int8", 16: "int16", 32: "int32", 64: "int64"}


# --- The device, as the ESI file describes it -----------------------------------------------------------------


@dataclass(frozen=True)
class EsiObject:
    """A mappable object entry of the ESI file: its name, ESI data type and width in bits."""

    name: str
    esi_type: str
    bits: int | None


@dataclass
class DeviceDescription:
    """What the slave_config needs to know about one device of the ESI file."""

    name: str
    vendor_id: int
    product_code: int
    assign_activate: int
    objects: dict[tuple[int, int], EsiObject] = field(default_factory=dict)


def parse_number(text: str | None) -> int | None:
    """Read a number in any notation of ESI files ('#x1A', '0x1A' or '26'). None when there is no number."""
    if text is None:
        return None
    text = text.strip()
    try:
        if text[:2].lower() in ("#x", "0x"):
            return int(text[2:], 16)
        return int(text)
    except ValueError:
        return None


def child_text(element: ET.Element, path: str) -> str:
    """Text of the element at `path`, stripped. Empty when it is missing."""
    return (element.findtext(path) or "").strip()


def child_bits(element: ET.Element, path: str) -> int | None:
    """Bit count of the element at `path`. None when it is missing."""
    text = child_text(element, path)
    return int(text) if text else None


def find_device(
    root: ET.Element, product: int, revision: int | None
) -> tuple[ET.Element, str]:
    """Find the device with this product code, and this revision when one is given. Returns it and its name."""
    for device in root.findall(".//Devices/Device"):
        identity = device.find("Type")
        if identity is None or parse_number(identity.get("ProductCode")) != product:
            continue
        if revision is None or parse_number(identity.get("RevisionNo")) == revision:
            return device, (identity.text or "").strip()
    wanted = (
        f"product code {product:#x}"
        if revision is None
        else f"product code {product:#x}, revision {revision:#x}"
    )
    raise SystemExit(f"No device with {wanted} in the ESI file")


def predefined_pdo_objects(device: ET.Element) -> dict[tuple[int, int], EsiObject]:
    """The entries of the device's predefined RxPDOs and TxPDOs."""
    found = {}
    for pdo in (*device.findall("RxPdo"), *device.findall("TxPdo")):
        for entry in pdo.findall("Entry"):
            index = parse_number(entry.findtext("Index"))
            if index is not None:
                sub_index = int(child_text(entry, "SubIndex") or 0)
                found[(index, sub_index)] = EsiObject(
                    child_text(entry, "Name"),
                    child_text(entry, "DataType"),
                    child_bits(entry, "BitLen"),
                )
    return found


def record_layouts(
    scopes: list[ET.Element],
) -> dict[str, dict[int, tuple[int | None, str]]]:
    """The mappable sub-items of every record type, as {type name: {sub_index: (bits, ESI type)}}.

    A type defined in more than one scope keeps its first definition.
    """
    layouts = {}
    for scope in scopes:
        for data_type in scope.findall(".//DataTypes/DataType"):
            name = child_text(data_type, "Name")
            if not name or name in layouts:
                continue
            mappable = {
                int(child_text(item, "SubIdx")): (
                    child_bits(item, "BitSize"),
                    child_text(item, "Type"),
                )
                for item in data_type.findall("SubItem")
                if child_text(item, "SubIdx") and child_text(item, "Flags/PdoMapping")
            }
            if mappable:
                layouts[name] = mappable
    return layouts


def dictionary_objects(scopes: list[ET.Element]) -> dict[tuple[int, int], EsiObject]:
    """The mappable objects of the object dictionaries in `scopes`.

    A record contributes each of its mappable sub-items. A plain object contributes sub-index 0 when it is mappable
    itself. The first scope that has an entry wins.
    """
    layouts = record_layouts(scopes)
    found = {}
    for scope in scopes:
        for obj in scope.findall(".//Objects/Object"):
            index = parse_number(obj.findtext("Index"))
            if index is None:
                continue
            name, type_name = child_text(obj, "Name"), child_text(obj, "Type")
            if type_name in layouts:
                for sub_index, (bits, esi_type) in layouts[type_name].items():
                    found.setdefault(
                        (index, sub_index), EsiObject(name, esi_type, bits)
                    )
            elif child_text(obj, "Flags/PdoMapping"):
                found.setdefault(
                    (index, 0), EsiObject(name, type_name, child_bits(obj, "BitSize"))
                )
    return found


def distributed_clock_activation(device: ET.Element) -> int:
    """AssignActivate of the OpMode named 'DC'. Else the first non-zero one, else DEFAULT_ASSIGN_ACTIVATE."""
    values = {}
    for op_mode in device.findall(".//Dc/OpMode"):
        value = parse_number(op_mode.findtext("AssignActivate"))
        if value:
            values.setdefault(child_text(op_mode, "Name").upper(), value)
    if "DC" in values:
        return values["DC"]
    return next(iter(values.values()), DEFAULT_ASSIGN_ACTIVATE)


def load_device(path: str, product: int, revision: int | None) -> DeviceDescription:
    """Read everything about one device from the ESI file at `path`.

    The device's predefined PDOs come first. SOMANET keeps its vendor objects in module dictionaries, so the
    dictionaries of the device and of all modules fill in what the PDOs do not have.
    """
    root = ET.parse(path).getroot()
    device, name = find_device(root, product, revision)
    vendor_id = parse_number(root.findtext(".//Vendor/Id"))
    if vendor_id is None:
        raise SystemExit("The ESI file gives no vendor ID")
    description = DeviceDescription(
        name=name,
        vendor_id=vendor_id,
        product_code=product,
        assign_activate=distributed_clock_activation(device),
        objects=predefined_pdo_objects(device),
    )
    for key, obj in dictionary_objects(
        [device, *root.findall(".//Modules/Module")]
    ).items():
        description.objects.setdefault(key, obj)
    return description


def slave_type(obj: EsiObject, where: str) -> str:
    """The slave_config type of an ESI object. `where` names the object in messages.

    A string of 64 bits travels as one real64. A vendor type of a common width falls back to the signed integer of
    that width, with a warning. Anything else stops the script.
    """
    esi_type = obj.esi_type.upper()
    if esi_type in SLAVE_TYPES:
        return SLAVE_TYPES[esi_type]
    if esi_type.startswith("STRING") and obj.bits == 64:
        return "real64"
    if obj.bits in SIGNED_TYPE_OF_WIDTH:
        fallback = SIGNED_TYPE_OF_WIDTH[obj.bits]
        print(
            f"Warning: {where} has the ESI type '{obj.esi_type}', written as {fallback}",
            file=sys.stderr,
        )
        return fallback
    raise SystemExit(
        f"{where} has the ESI type '{obj.esi_type}' of {obj.bits} bits, which no slave_config type fits"
    )


# --- Selecting and writing ------------------------------------------------------------------------------------


def address(index: int, sub_index: int) -> str:
    """An object address as written in messages, for example 0x2060:1."""
    return f"{index:#06x}:{sub_index}"


def layout_for(variant: str) -> list[tuple[Pdo, list[PdoEntry]]]:
    """Each PDO of the layout with the entries `variant` maps. A PDO without any entry is left out."""
    selected = []
    for pdo in PDO_LAYOUT:
        entries = [entry for entry in pdo.entries if variant in entry.variants]
        if entries:
            selected.append((pdo, entries))
    return selected


def sdos_for(variant: str) -> list[StartupSdo]:
    """The startup SDOs of `variant`, active or not, in writing order."""
    return [sdo for sdo in STARTUP_SDOS if variant in sdo.variants]


def missing_objects(device: DeviceDescription, variant: str) -> list[str]:
    """Addresses of the layout entries that the device can map neither from a PDO nor from a dictionary."""
    return [
        address(entry.index, entry.sub_index)
        for _, entries in layout_for(variant)
        for entry in entries
        if (entry.index, entry.sub_index) not in device.objects
    ]


def yaml_value(value: int | float) -> str:
    """A number as YAML writes it, with NaN as .nan."""
    if isinstance(value, float) and math.isnan(value):
        return ".nan"
    return str(value)


def comment_block(text: str) -> list[str]:
    """`text` as YAML comment lines, a blank line becoming a bare '#'."""
    return [f"# {line}" if line else "#" for line in text.rstrip("\n").split("\n")]


def header_lines(
    device: DeviceDescription, variant: Variant, esi_name: str
) -> list[str]:
    """The opening comments and the device settings."""
    lines = [
        f"# Slave_config of the {device.name}, {variant.title}.",
        f"# Generated by esi_to_slave_config.py from {esi_name}.",
    ]
    lines += comment_block(NOTES["units"])
    if "modes" in variant.notes:
        lines += comment_block(NOTES["modes"])
    settings = [
        ("vendor_id", f"{device.vendor_id:#010x}", None),
        ("product_id", f"{device.product_code:#010x}", None),
        (
            "assign_activate",
            f"{device.assign_activate:#06x}",
            "distributed clock activation from the ESI file",
        ),
    ]
    for key, value, note in settings:
        lines.append(f"{key}: {value}" + (f"  # {note}" if note else ""))
    return lines


def sdo_lines(variant: Variant) -> list[str]:
    """The notes on the startup SDOs and the `sdo` list."""
    lines = [""]
    for topic in ("startup", "impedance"):
        if topic in variant.notes:
            if lines[-1] != "":
                lines.append("#")
            lines += comment_block(NOTES[topic])
    for section, si in (("sdo", False), ("sdo_si", True)):
        sdos = [sdo for sdo in sdos_for(variant.name) if sdo.si == si]
        if not sdos:
            continue
        lines.append(f"{section}:")
        for sdo in sdos:
            key = "si_value" if sdo.si else "value"
            fields = (
                f"index: {sdo.index:#06x}, sub_index: {sdo.sub_index}, type: {sdo.type}, "
                f"{key}: {yaml_value(sdo.value)}"
            )
            prefix = "  - " if sdo.active else "  # - "
            lines += [f"  # {sdo.note}", f"{prefix}{{ {fields} }}"]
    return lines


def entry_lines(
    entry: PdoEntry, direction: str, device: DeviceDescription
) -> list[str]:
    """One PDO entry as a flow mapping, after its note as a comment."""
    where = address(entry.index, entry.sub_index)
    fields = [
        f"index: {entry.index:#06x}",
        f"sub_index: {entry.sub_index}",
        f"type: {slave_type(device.objects[(entry.index, entry.sub_index)], where)}",
    ]
    if entry.interface is not None:
        fields.append(f"{direction}_interface: {entry.interface}")
    if entry.scaled:
        fields.append("factor: 1.0")
    if entry.default is not None:
        fields.append(f"default: {yaml_value(entry.default)}")
    return [f"      # {entry.note}", "      - { " + ", ".join(fields) + " }"]


def pdo_lines(device: DeviceDescription, variant: Variant) -> list[str]:
    """The `rpdo` and `tpdo` lists."""
    lines = []
    for key, direction in (("rpdo", COMMAND), ("tpdo", STATE)):
        lines.append(f"{key}:")
        for pdo, entries in layout_for(variant.name):
            if pdo.direction == direction:
                lines += [f"  - index: {pdo.index:#06x}", "    channels:"]
                for entry in entries:
                    lines += entry_lines(entry, direction, device)
    return lines


def render(device: DeviceDescription, variant: Variant, esi_name: str) -> str:
    """The complete slave_config text."""
    lines = (
        header_lines(device, variant, esi_name)
        + sdo_lines(variant)
        + [""]
        + pdo_lines(device, variant)
    )
    return "\n".join(lines) + "\n"


# --- Command line ---------------------------------------------------------------------------------------------


def parse_arguments(argv: list[str] | None) -> argparse.Namespace:
    """The command line options."""
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--esi", required=True, help="ESI file of the drive's firmware")
    parser.add_argument(
        "--product", required=True, help="product code of the device, for example 0x302"
    )
    parser.add_argument(
        "--revision",
        help="revision number of the device (default: the first with the product code)",
    )
    parser.add_argument("--out", required=True, help="where to write the slave_config")
    parser.add_argument(
        "--csp-only",
        dest="impedance",
        action="store_false",
        help="write a CSP-only slave_config without the impedance objects, starting in mode 8",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    """Load the device from the ESI file, check that it can map the layout, and write its slave_config."""
    args = parse_arguments(argv)
    product = parse_number(args.product)
    revision = parse_number(args.revision) if args.revision is not None else None
    if product is None or (args.revision is not None and revision is None):
        raise SystemExit("--product and --revision take a number, for example 0x302")
    variant = VARIANTS[IMPEDANCE if args.impedance else CSP]
    device = load_device(args.esi, product, revision)
    missing = missing_objects(device, variant.name)
    if missing:
        raise SystemExit(
            f"The {device.name} cannot map these objects of the layout: "
            + ", ".join(missing)
        )
    text = render(device, variant, os.path.basename(args.esi))
    with open(args.out, "w", encoding="utf-8") as stream:
        stream.write(text)
    print(f"Wrote the slave_config {variant.title} of the {device.name} to {args.out}")


if __name__ == "__main__":
    main()
