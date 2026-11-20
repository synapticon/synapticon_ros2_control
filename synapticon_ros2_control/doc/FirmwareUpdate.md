# Updating the drive firmware without OBLAC Drives

How to flash new firmware onto a SOMANET drive with the IgH EtherCAT master command line tool (`ethercat`),
using File access over EtherCAT (FoE).

> [!IMPORTANT]
> Synapticon recommends their commissioning tool **OBLAC Drives** for firmware updates.
> It checks that the firmware matches the drive's hardware and version, and a third-party master cannot do that.
> With the steps below, picking the right firmware package is your responsibility.
> If in doubt, ask Synapticon, in particular for Safe Motion drives.

All commands take `-p<pos>`, the slave's position on the bus (`-p0` is the first drive).
With several drives, repeat the steps for each position.

## Before you start

- **Nothing else may use the EtherCAT master.** Stop the ros2_control stack, OBLAC Drives, Motion Master
  and any other master application.
- **Do not cut the power while the firmware is transferred.**
- Check the current firmware version:
  ```bash
  ethercat upload -p0 --type string 0x100a 0   # for example v5.6.6
  ```

Some drives, for example the SOMANET Integro, also speak Ethernet over EtherCAT (EoE).
For each of them, the master creates a network interface on the PC, named like `eoe0s1`.
If NetworkManager takes it over, its traffic blocks the drive's message channel.
Reads and file transfers then fail at random.

Nothing here needs these interfaces, so tell NetworkManager once to ignore them:

```bash
printf '[keyfile]\nunmanaged-devices+=interface-name:eoe*\n' | sudo tee /etc/NetworkManager/conf.d/99-ethercat-eoe.conf
sudo nmcli general reload
```

If a drive has already stopped answering, bring its interface down and restart the drive's communication:

```bash
sudo ip link set eoe0s1 down
ethercat states -p0 INIT
ethercat states -p0 PREOP
```

## 1. Identify the drive

The firmware package is selected by the drive's **identifier**.
OBLAC Drives shows it, and without OBLAC you can read it from the drive's `.hardware_description` file.
That file is protected, so unlock the protected files first.
FoE needs the mailbox, so the drive must be in PREOP (or BOOT), not INIT:

```bash
ethercat states -p0 PREOP
ethercat foe_read -p0 fs-stackunlock=DD1317   # replies "protected files are unlocked"
ethercat foe_read -p0 .hardware_description
```

The output is JSON. Shortened example:

```json
{"fileVersion": "1.1.0", "device": {"keyId": "2332", "serialNumber": "8604-02-...",
 "name": "SOMANET Circulo 7 Safe Motion (CR7-48-24-ECANSM)", "id": "8604", "version": "02",
 "components": [...]}}
```

The identifier is made of three numbers, four on an Integro. The firmware bundle page shows them as columns:

| Page column | Field | Circulo 7 | ACTILINK-S |
|---|---|---|---|
| Firmware ID | `id` | `8604` | `6000` |
| Rev. | `version` | `02` | `05` |
| Enc. Key | `keyId` | `2332` | `2423` |
| Protocol | Integro only | none | `1` |

So the identifiers are `8604-02-2332` and `6000-05-2423-1`.
Go by these numbers, not by the product names on the page.

A drive built into a product, like the ACTILINK-S, has a second block named `assembly` next to `device`.
Then `id` and `version` come from `assembly`. The `keyId` still comes from `device`.

Integro packages exist once per fieldbus. The fourth number picks it: `0` EtherNet/IP, `1` EtherCAT, `2` PROFINET.

## 2. Back up the drive configuration

The drive stores its configuration (for example the commutation offset `0x2001` and the quick stop settings)
in files. Save all of them. First list them:

```bash
ethercat foe_read -p0 fs-getlist
```

Usually the list has `config.csv` and `plant_model.csv`.
A drive whose configuration was never saved has neither. Its setup is then in protected factory files,
such as `.factory_config`.

`foe_read` prints a file, so redirect it into a local file. Do this for every file in the list.
All later steps run from the directory that contains `drive_backup`, so do not change into it:

```bash
mkdir -p drive_backup
ethercat foe_read -p0 config.csv > drive_backup/config.csv
ethercat foe_read -p0 plant_model.csv > drive_backup/plant_model.csv
ethercat foe_read -p0 .hardware_description > drive_backup/hardware_description.json
ls -l drive_backup            # no file may be empty
```

Keep the backup in a permanent place, one directory per drive.

## 3. Get the firmware package

Download the package for your identifier from the [Synapticon firmware bundles](https://bundles.synapticon.com/latest/latest/packages),
under "Browse SOMANET Firmwares".
The package is named after the identifier, for example `package_SOMANET-Circulo-7_8604-02-2332_motion-drive_v5.6.11.zip`
or `package_ACTILINK_6000-05-2423-1_motion-drive_v5.6.11.zip`.
Unzip it. Depending on the drive, it contains:

- the firmware binary, for example `app_8604-02_motion-drive_v5.6.11_2332.bin`
- the SII (EEPROM) file, `somanet_cia402.sii`. Integro packages have none.
- the ESI as a nested zip, `SOMANET_CiA_402.xml.zip`. Unzip it too, you need it later to generate the slave_config.
- possibly a communication firmware, `com_<...>.bin`. Integro packages have one.

Also download the package of the version the drive runs now.
The firmware cannot be read back from the drive, so this package is your way back.

## 4. Write the SII

The SII comes first, then the firmware. Synapticon's own update tool, OBLAC Drives, does it in the same order, and
writes the package's SII at every firmware update.
Skip this step if the package has no SII file.
The SII must be written in **INIT**, not in BOOT:

```bash
ethercat states -p0 INIT
ethercat sii_write -p0 somanet_cia402.sii
```

`sii_write` checks the file for validity first. Its `-f` (`--force`) option skips that check,
only use it if Synapticon tells you to.

Then **power-cycle the drive**, so that it starts with the new SII.

## 5. Flash the firmware

Every file written over FoE needs the drive in BOOT. Switch to it and check that the drive got there:

```bash
ethercat states -p0 BOOT
ethercat slaves -p0           # must show BOOT
```

Transfer the firmware. The file must arrive on the drive under the name `app_firmware.bin`.
The packages name the binary after the drive and version instead, and `foe_write` uses the local file name,
so copy the binary to that name first:

```bash
cp app_<...>.bin app_firmware.bin
ethercat foe_write -p0 app_firmware.bin
```

If the package contains a `com_<...>.bin`, transfer it as well, under the name `com_firmware.bin`:

```bash
cp com_<...>.bin com_firmware.bin
ethercat foe_write -p0 com_firmware.bin
```

Wait until the command has returned. Do not power off the drive during the transfer.

## 6. Start the new firmware

Either power-cycle the drive, or switch it back through the states.
PREOP is as far as it goes: without a program that cycles the bus, the drive cannot hold OP.

```bash
ethercat states -p0 INIT
ethercat states -p0 PREOP
```

## 7. Check the result

```bash
ethercat upload -p0 --type string 0x100a 0                 # the new version
ethercat slaves -p0 -v                                     # product code and revision number
ethercat foe_read -p0 config.csv | diff - drive_backup/config.csv
```

If `config.csv` differs, find out from Synapticon whether the old file is compatible with the new firmware
before you write it back. It is written in BOOT, the same way as the firmware:

```bash
ethercat states -p0 BOOT
ethercat foe_write -p0 drive_backup/config.csv
```

## 8. Afterwards

- **Regenerate the slave_config** from the new package's ESI, as described in the README
  ("Generate a slave_config"). Use the revision number reported in step 7,
  since the PDO mapping can change between firmware versions.
- **Re-commission what a flash resets.** For impedance this is the torque-sensor type,
  see "Commission the drive for impedance" in the README.
- Check that bring-up no longer complains about the firmware version.

The drive also keeps its own copy of the ESI, and the update does not replace it.
The HALs never read that copy, only tools that take the ESI from the drive do. So updating it is optional.

To update it, split the new `SOMANET_CiA_402.xml.zip` into parts of 9000 bytes, the size the drive uses,
and write them in BOOT:

```bash
split -b 9000 -d -a 3 SOMANET_CiA_402.xml.zip SOMANET_CiA_402.xml.zip.part
ethercat states -p0 BOOT
for part in SOMANET_CiA_402.xml.zip.part*; do ethercat foe_write -p0 "$part"; done
ethercat states -p0 INIT
ethercat states -p0 PREOP
```

Compare with `fs-getlist` first: if the drive holds more parts than you write, the extra old parts stay behind.

## Sources

- Synapticon: "Drive Operations through File Access over EtherCAT (FoE)", for SOMANET Circulo and SOMANET Node.
- [Synapticon firmware bundles](https://bundles.synapticon.com/latest/latest/packages)
- [Circulo downloads](https://doc.synapticon.com/circulo/technical_specs/downloads_circulo.html) (ESI files)
