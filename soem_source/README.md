# soem_source

The [SOEM](https://github.com/OpenEtherCATsociety/SOEM) (Simple Open EtherCAT Master) library, vendored.
The package builds it for Linux as a static library with position-independent code and exports the CMake target
`soem_source::soem`. The headers are installed into `include/soem_source`.

## Source

The files in `src/soem`, `src/osal` and `src/oshw` are the Linux library sources of upstream SOEM at commit `42aadc1`
(2023-10-13), which lies between the tags `v1.4.0` and `v2.0.0`.
They are unmodified apart from whitespace in six files, all in `src/soem`: `ethercatconfig.c`, `ethercateoe.c`,
`ethercateoe.h`, `ethercatmain.c`, `ethercatmain.h` and `ethercatsoe.h`.

Do not update these files to SOEM 2.x. From 2.0 on, SOEM is licensed under GPLv3 or a commercial license, without the
linking exception below.

## License

SOEM is licensed under the GNU General Public License version 2 with a linking exception, see [LICENSE](LICENSE).
The GPL text is in [GPL-2.0.txt](GPL-2.0.txt).
The exception means that linking SOEM into other code does not by itself make that code GPL, while the SOEM source
must stay available.

The [LICENSE](LICENSE) also states that using SOEM for an EtherCAT network master requires an EtherCAT Master License
from Beckhoff Automation GmbH.
