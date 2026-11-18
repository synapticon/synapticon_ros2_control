# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Elevate Robotics Inc
# Copyright (c) 2025-2026 Synapticon GmbH
# Copyright (c) 2026 b-robotized GmbH

"""Unit tests of esi_to_slave_config.py: the layout tables, reading an ESI file, type resolution and the output.

The ESI file of the tests is written inline below. It is a small stand-in for a SOMANET firmware ESI file: two
revisions of the Circulo SMM and one other device, predefined PDOs for the CiA 402 objects, and a module dictionary
with the vendor objects, as the real files have them.
"""

import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[2]
BUNDLED_EXAMPLE = PACKAGE / "config" / "example_somanet_circulo_safe_motion.yaml"
ESI_NAME = "SOMANET_CiA_402_v5.6.11.xml"


def import_generator():
    """Load the script as a module, since it is installed as a program and not as a package."""
    spec = importlib.util.spec_from_file_location(
        "esi_to_slave_config", PACKAGE / "scripts" / "esi_to_slave_config.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gen = import_generator()

# --- The ESI file of the tests --------------------------------------------------------------------------------

# Predefined PDO entries of the device: (PDO tag, index, ESI type, bits).
PREDEFINED = [
    ("RxPdo", 0x6040, "UINT", 16),
    ("RxPdo", 0x6060, "SINT", 8),
    ("RxPdo", 0x607A, "DINT", 32),
    ("RxPdo", 0x60FF, "DINT", 32),
    ("TxPdo", 0x6041, "UINT", 16),
    ("TxPdo", 0x6061, "SINT", 8),
    ("TxPdo", 0x6064, "DINT", 32),
    ("TxPdo", 0x606C, "DINT", 32),
    ("TxPdo", 0x6077, "INT", 16),
]

# Plain objects of the module dictionary: (index, ESI type, bits, mappable).
PLAIN_OBJECTS = [
    (0x1000, "UDINT", 32, False),
    (0x2053, "DINT", 32, True),
    (0x2054, "UDINT", 32, True),
    (0x2052, "DINT", 32, True),
    (0x603F, "UINT", 16, True),
    (0x6079, "UDINT", 32, True),
    # Also a predefined PDO entry, with another type here, to show which source wins.
    (0x6040, "UDINT", 32, True),
]

# Records of the module dictionary: index -> [(sub_index, ESI type, bits, mappable)].
RECORDS = {
    0x2060: [
        (0, "USINT", 8, False),
        (1, "DINT", 32, True),
        (2, "DINT", 32, True),
        (3, "REAL", 32, False),
    ],
    0x2030: [(0, "USINT", 8, False), (1, "DINT", 32, True)],
    0x2031: [(0, "USINT", 8, False), (1, "DINT", 32, True)],
    0x2041: [(0, "USINT", 8, False), (1, "DINT", 32, True)],
    0x203F: [(0, "USINT", 8, False), (1, "STRING(8)", 64, True)],
}

DC_MODES = [("Freerun", "#x0"), ("DC", "#x300")]


def pdo_entry_xml(index, esi_type, bits):
    """One entry of a predefined PDO."""
    return (
        f"<Entry><Index>#x{index:04X}</Index><SubIndex>0</SubIndex><BitLen>{bits}</BitLen>"
        f"<Name>Object {index:04X}</Name><DataType>{esi_type}</DataType></Entry>"
    )


def device_xml(revision, name, skip, dc_modes):
    """A Circulo SMM device element without the objects in `skip`."""
    pdos = ""
    for tag in ("RxPdo", "TxPdo"):
        entries = "".join(
            pdo_entry_xml(index, esi_type, bits)
            for pdo_tag, index, esi_type, bits in PREDEFINED
            if pdo_tag == tag and index not in skip
        )
        pdos += f"<{tag}>{entries}</{tag}>"
    modes = "".join(
        f"<OpMode><Name>{mode}</Name><AssignActivate>{value}</AssignActivate></OpMode>"
        for mode, value in dc_modes
    )
    return (
        f'<Device><Type ProductCode="#x302" RevisionNo="{revision}">{name}</Type>'
        f"{pdos}<Dc>{modes}</Dc></Device>"
    )


def mapping_flag(mappable, direction="t"):
    """The Flags element of a mappable item, nothing for one that is not."""
    return f"<Flags><PdoMapping>{direction}</PdoMapping></Flags>" if mappable else ""


def dictionary_xml(skip):
    """The module dictionary, without the objects in `skip`."""
    types = ""
    objects = ""
    for index, items in RECORDS.items():
        if index in skip:
            continue
        sub_items = "".join(
            f"<SubItem><SubIdx>{sub}</SubIdx><Name>Item {sub}</Name><Type>{esi_type}</Type>"
            f"<BitSize>{bits}</BitSize>{mapping_flag(mappable)}</SubItem>"
            for sub, esi_type, bits, mappable in items
        )
        types += f"<DataType><Name>DT{index:04X}</Name>{sub_items}</DataType>"
        objects += (
            f"<Object><Index>#x{index:04X}</Index><Name>Record {index:04X}</Name>"
            f"<Type>DT{index:04X}</Type></Object>"
        )
    for index, esi_type, bits, mappable in PLAIN_OBJECTS:
        if index not in skip:
            objects += (
                f"<Object><Index>#x{index:04X}</Index><Name>Plain {index:04X}</Name><Type>{esi_type}</Type>"
                f"<BitSize>{bits}</BitSize>{mapping_flag(mappable, 'r')}</Object>"
            )
    return (
        "<Modules><Module><Type>CiA402 Dictionary</Type><Profile><Dictionary>"
        f"<DataTypes>{types}</DataTypes><Objects>{objects}</Objects>"
        "</Dictionary></Profile></Module></Modules>"
    )


def esi_text(skip=(), vendor="#x000022D2", dc_modes=DC_MODES):
    """The whole ESI file: an early revision of the Circulo SMM first, then the current one, then a Node."""
    vendor_xml = f"<Vendor><Id>{vendor}</Id></Vendor>" if vendor else "<Vendor/>"
    return (
        '<?xml version="1.0" encoding="utf-8"?><EtherCATInfo>'
        f"{vendor_xml}<Descriptions><Devices>"
        f'{device_xml("#x11000001", "SOMANET Circulo SMM, early revision", skip, dc_modes)}'
        f'{device_xml("#x11000003", "SOMANET Circulo SMM", skip, dc_modes)}'
        '<Device><Type ProductCode="#x201" RevisionNo="#x11000003">SOMANET Node</Type></Device>'
        f"</Devices></Descriptions>{dictionary_xml(skip)}</EtherCATInfo>"
    )


@pytest.fixture
def esi_file(tmp_path):
    """Write the ESI file of the tests and return its path."""
    path = tmp_path / ESI_NAME
    path.write_text(esi_text(), encoding="utf-8")
    return path


def generate(tmp_path, esi_path, *extra):
    """Run the script on `esi_path` for the current Circulo SMM and return the written text."""
    out = tmp_path / "out.yaml"
    gen.main(
        [
            "--esi",
            str(esi_path),
            "--product",
            "0x302",
            "--revision",
            "0x11000003",
            "--out",
            str(out),
            *extra,
        ]
    )
    return out.read_text(encoding="utf-8")


def channels_of(config, kind):
    """All channels of the `rpdo` or `tpdo` list."""
    return [channel for pdo in config[kind] for channel in pdo["channels"]]


# --- The layout tables ----------------------------------------------------------------------------------------


def test_command_pdos_are_rxpdos_and_state_pdos_are_txpdos():
    for pdo in gen.PDO_LAYOUT:
        if pdo.direction == gen.COMMAND:
            assert 0x1600 <= pdo.index <= 0x17FF
        else:
            assert pdo.direction == gen.STATE
            assert 0x1A00 <= pdo.index <= 0x1BFF


@pytest.mark.parametrize("variant", sorted(gen.VARIANTS))
def test_a_variant_maps_each_object_once_and_binds_each_interface_once(variant):
    objects = [
        (e.index, e.sub_index)
        for _, entries in gen.layout_for(variant)
        for e in entries
    ]
    assert len(objects) == len(set(objects))
    for direction in (gen.COMMAND, gen.STATE):
        names = [
            e.interface
            for pdo, entries in gen.layout_for(variant)
            if pdo.direction == direction
            for e in entries
            if e.interface
        ]
        assert len(names) == len(set(names))


def commands_of(variant):
    """The command entries of `variant` by interface name."""
    return {
        e.interface: e
        for pdo, entries in gen.layout_for(variant)
        if pdo.direction == gen.COMMAND
        for e in entries
        if e.interface
    }


def test_the_csp_variant_has_no_impedance_interfaces_and_starts_in_csp():
    csp = commands_of(gen.CSP)
    full = commands_of(gen.IMPEDANCE)
    impedance_only = {"stiffness", "damping", "joint_torque_offset", "max_joint_torque"}
    assert impedance_only <= set(full)
    assert not impedance_only & set(csp)
    assert csp["mode_of_operation"].default == 8
    assert full["mode_of_operation"].default == -6


@pytest.mark.parametrize("variant", sorted(gen.VARIANTS))
def test_startup_sdos_have_valid_types_and_appear_once(variant):
    sdos = gen.sdos_for(variant)
    assert len({(s.index, s.sub_index) for s in sdos}) == len(sdos)
    assert {s.type for s in sdos} <= set(gen.SLAVE_TYPES.values())
    assert all(note in gen.NOTES for note in gen.VARIANTS[variant].notes)


def test_only_the_impedance_variant_seeds_the_impedance_objects():
    def active(variant):
        return {(s.index, s.sub_index) for s in gen.sdos_for(variant) if s.active}

    assert active(gen.CSP) == {(0x605A, 0)}
    assert active(gen.IMPEDANCE) == {
        (0x605A, 0),
        (0x2060, 3),
    }


# --- Reading numbers and types --------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "text, value",
    [
        ("#x302", 0x302),
        ("#X1a", 0x1A),
        ("0x11000003", 0x11000003),
        (" 26 ", 26),
        ("#xG1", None),
        ("", None),
        (None, None),
    ],
)
def test_numbers_are_read_in_every_esi_notation(text, value):
    assert gen.parse_number(text) == value


def test_sub_indexes_and_bit_counts_are_read_in_every_esi_notation():
    device = ET.fromstring(
        "<Device><TxPdo><Entry><Index>#x2060</Index><SubIndex>#x2</SubIndex><BitLen>#x20</BitLen>"
        "<Name>Gain</Name><DataType>DINT</DataType></Entry></TxPdo></Device>"
    )
    assert gen.predefined_pdo_objects(device) == {
        (0x2060, 2): gen.EsiObject("Gain", "DINT", 32)
    }
    dictionary = ET.fromstring(
        "<Dictionary><DataTypes><DataType><Name>DT2060</Name>"
        "<SubItem><SubIdx>#x1</SubIdx><Type>DINT</Type><BitSize>#x20</BitSize>"
        "<Flags><PdoMapping>t</PdoMapping></Flags></SubItem>"
        "</DataType></DataTypes></Dictionary>"
    )
    assert gen.record_layouts([dictionary]) == {"DT2060": {1: (32, "DINT")}}


@pytest.mark.parametrize(
    "esi_type, expected",
    [("UINT", "uint16"), ("sint", "int8"), ("LREAL", "real64"), ("BOOL", "bool")],
)
def test_base_types_have_a_slave_config_type(esi_type, expected):
    assert gen.slave_type(gen.EsiObject("x", esi_type, None), "0x1000:0") == expected


def test_an_eight_character_string_travels_as_real64():
    assert (
        gen.slave_type(gen.EsiObject("text", "STRING(8)", 64), "0x203f:1") == "real64"
    )


def test_a_vendor_type_of_a_common_width_becomes_a_signed_integer_with_a_warning(
    capsys,
):
    assert (
        gen.slave_type(gen.EsiObject("vendor", "DT_VENDOR", 16), "0x2100:0") == "int16"
    )
    assert "0x2100:0" in capsys.readouterr().err


@pytest.mark.parametrize(
    "esi_type, bits", [("DT_VENDOR", 12), ("STRING(4)", 24), ("DT_VENDOR", None)]
)
def test_a_type_without_a_slave_config_type_stops(esi_type, bits):
    with pytest.raises(SystemExit, match="no slave_config type fits"):
        gen.slave_type(gen.EsiObject("odd", esi_type, bits), "0x2100:0")


# --- Reading the device ---------------------------------------------------------------------------------------


def test_the_revision_picks_the_device(esi_file):
    assert gen.load_device(esi_file, 0x302, 0x11000003).name == "SOMANET Circulo SMM"
    assert (
        gen.load_device(esi_file, 0x302, 0x11000001).name
        == "SOMANET Circulo SMM, early revision"
    )


def test_without_a_revision_the_first_device_with_the_product_code_is_used(esi_file):
    assert (
        gen.load_device(esi_file, 0x302, None).name
        == "SOMANET Circulo SMM, early revision"
    )


@pytest.mark.parametrize(
    "product, revision, message",
    [
        (0x401, None, "product code 0x401 in"),
        (0x302, 0x11000002, "revision 0x11000002"),
    ],
)
def test_a_device_not_in_the_file_stops(esi_file, product, revision, message):
    with pytest.raises(SystemExit, match=message):
        gen.load_device(esi_file, product, revision)


def test_the_device_identity_comes_from_the_file(esi_file):
    device = gen.load_device(esi_file, 0x302, 0x11000003)
    assert (device.vendor_id, device.product_code, device.assign_activate) == (
        0x22D2,
        0x302,
        0x300,
    )


def test_a_file_without_a_vendor_id_stops(tmp_path):
    path = tmp_path / ESI_NAME
    path.write_text(esi_text(vendor=None), encoding="utf-8")
    with pytest.raises(SystemExit, match="vendor ID"):
        gen.load_device(path, 0x302, None)


def test_a_predefined_pdo_entry_wins_over_the_dictionary(esi_file):
    objects = gen.load_device(esi_file, 0x302, 0x11000003).objects
    assert objects[(0x6040, 0)].esi_type == "UINT"


def test_a_record_contributes_only_its_mappable_sub_items(esi_file):
    objects = gen.load_device(esi_file, 0x302, 0x11000003).objects
    assert {(0x2060, 1), (0x2060, 2), (0x203F, 1)} <= set(objects)
    assert not {(0x2060, 0), (0x2060, 3)} & set(objects)
    assert objects[(0x203F, 1)] == gen.EsiObject("Record 203F", "STRING(8)", 64)


def test_a_plain_object_counts_only_when_it_is_mappable(esi_file):
    objects = gen.load_device(esi_file, 0x302, 0x11000003).objects
    assert objects[(0x2054, 0)] == gen.EsiObject("Plain 2054", "UDINT", 32)
    assert (0x1000, 0) not in objects


@pytest.mark.parametrize(
    "dc_modes, expected",
    [
        ([("Synchron", "#x100"), ("DC", "#x300")], 0x300),
        ([("Freerun", "#x0"), ("Synchron", "#x700")], 0x700),
        ([("Freerun", "#x0")], gen.DEFAULT_ASSIGN_ACTIVATE),
        ([], gen.DEFAULT_ASSIGN_ACTIVATE),
    ],
)
def test_the_distributed_clock_activation_prefers_the_dc_op_mode(
    tmp_path, dc_modes, expected
):
    path = tmp_path / ESI_NAME
    path.write_text(esi_text(dc_modes=dc_modes), encoding="utf-8")
    assert gen.load_device(path, 0x302, 0x11000003).assign_activate == expected


# --- Writing the slave_config ---------------------------------------------------------------------------------


def test_the_bundled_example_is_what_the_script_writes_for_its_device(
    tmp_path, esi_file
):
    assert generate(tmp_path, esi_file) == BUNDLED_EXAMPLE.read_text(encoding="utf-8")


def test_the_output_carries_the_types_of_the_esi_file_and_factors_only_where_scaled(
    tmp_path, esi_file
):
    config = yaml.safe_load(generate(tmp_path, esi_file))
    assert (config["vendor_id"], config["product_id"], config["assign_activate"]) == (
        0x22D2,
        0x302,
        0x300,
    )
    by_object = {
        (c["index"], c["sub_index"]): c
        for c in channels_of(config, "rpdo") + channels_of(config, "tpdo")
    }
    assert by_object[(0x6077, 0)]["type"] == "int16"
    assert by_object[(0x203F, 1)]["type"] == "real64"
    assert "factor" not in by_object[(0x6041, 0)]
    assert by_object[(0x6064, 0)]["factor"] == 1.0
    assert math.isnan(by_object[(0x607A, 0)]["default"])
    assert all("default" not in c for c in channels_of(config, "tpdo"))
    assert all("command_interface" not in c for c in channels_of(config, "tpdo"))


def test_the_csp_only_file_has_no_impedance_objects(tmp_path, esi_file):
    config = yaml.safe_load(generate(tmp_path, esi_file, "--csp-only"))
    commands = {c.get("command_interface"): c for c in channels_of(config, "rpdo")}
    assert not {
        "stiffness",
        "damping",
        "joint_torque_offset",
        "max_joint_torque",
    } & set(commands)
    assert commands["mode_of_operation"]["default"] == 8
    assert [(s["index"], s["value"]) for s in config["sdo"]] == [(0x605A, 6)]


def test_disabled_startup_sdos_are_written_as_comments(tmp_path, esi_file):
    text = generate(tmp_path, esi_file)
    assert (
        "\nsdo_si:\n  # quick stop deceleration in rad/s^2 on the position shaft, see the note on 0x605A\n"
        "  # - { index: 0x6085, sub_index: 0, type: uint32, si_value: 200.0 }\n" in text
    )
    config = yaml.safe_load(text)
    assert all(s["index"] not in (0x6085, 0x2150) for s in config["sdo"])
    assert config["sdo_si"] is None  # the section holds only comments


def test_a_layout_object_the_device_lacks_stops_and_writes_nothing(tmp_path):
    path = tmp_path / ESI_NAME
    path.write_text(esi_text(skip={0x2052, 0x2041}), encoding="utf-8")
    with pytest.raises(SystemExit, match="0x2041:1, 0x2052:0"):
        generate(tmp_path, path)
    assert not (tmp_path / "out.yaml").exists()


def test_the_csp_only_file_does_not_need_the_impedance_objects(tmp_path):
    path = tmp_path / ESI_NAME
    path.write_text(esi_text(skip={0x2060, 0x2053, 0x2054}), encoding="utf-8")
    assert "stiffness" not in generate(tmp_path, path, "--csp-only")
    with pytest.raises(SystemExit, match="0x2053:0"):
        generate(tmp_path, path)


@pytest.mark.parametrize(
    "option, value", [("--product", "circulo"), ("--revision", "latest")]
)
def test_a_product_or_revision_that_is_no_number_stops(
    tmp_path, esi_file, option, value
):
    arguments = {
        "--esi": str(esi_file),
        "--product": "0x302",
        "--out": str(tmp_path / "out.yaml"),
        option: value,
    }
    with pytest.raises(SystemExit, match="take a number"):
        gen.main([item for pair in arguments.items() for item in pair])


def test_the_script_says_what_it_wrote(tmp_path, esi_file, capsys):
    generate(tmp_path, esi_file, "--csp-only")
    assert "for CSP of the SOMANET Circulo SMM" in capsys.readouterr().out
