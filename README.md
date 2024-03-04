<div align="center">

<img src="https://upload.wikimedia.org/wikipedia/commons/b/ba/TexasInstruments-Logo.svg" width="150"><br/>
# Motor Control SDK

[Introduction](#introduction) | [Features](#features) | [Overview](#overview) | [Learn](#learn) | [Usage](#usage) | [Contribute](#contributing-to-the-project)

</div>

## Introduction

Motor Control SDK enables real-time communication for position and current sense from motors, and real-time control libraries for ARM R5F CPU and related peripherals for TI processors.

Real-time communication with encoders and current sensing is typically handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low level firmware. The PRU-ICSS frees up the main ARM cores in the device for other functions, such as control and data processing.

The devices supported by Motor Control SDK currently include

- [AM2431](https://www.ti.com/product/AM2431), [AM2432](https://www.ti.com/product/AM2432), [AM2434](https://www.ti.com/product/AM2434)
- [AM2631](https://www.ti.com/product/AM2631), [AM2632](https://www.ti.com/product/AM2632), [AM2634](https://www.ti.com/product/AM2634)

## Features

- Out of Box Application Examples
  - Position Sense Encoders
  - Current Sense (SDFM)
  - Real Time Libraries (Digital control Library, Transforms)
  - PRUICSS PWM

- Protocol stacks and middleware
  - Various Industrial Protocol Stacks

- Firmware
  - Firmware for Position Sense Encoders and Current Sense
  - Industrial protocol firmware

#### Supported HOST environments

- Windows 10 64bit
- Ubuntu 18.04 64bit

### Clone and build from GIT

#### Repo Tool Setup

Motor Control SDK needs Industrial Communicaions SDK components and MCU+ SDK components (in multiple repositories) and dependencies
(like compiler, CCS and other tools). We use repo tool from Google to manage these
multiple repositories. Currently there is no support for native windows shells like
CMD or Powershell. This will be added at a later point. Windows users can rely on
Git Bash for the repo setup. Follow the below mentioned steps to setup repo tool:

Make sure [python3 is installed](https://wiki.python.org/moin/BeginnersGuide/Download) and is in your OS path.

- Linux:
  Do the following in terminal
  ```bash
  curl https://storage.googleapis.com/git-repo-downloads/repo > ~/bin/repo
  chmod a+x ~/bin/repo
  echo "PATH=$HOME/bin:$PATH" >> ~/.bashrc
  source ~/.bashrc
  ```

- Git-Bash (Windows)
  Make sure that python 3 is callable as `python` from CMD. Do below in Git-Bash
  ```bash
  mkdir -p /c/ti
  curl https://storage.googleapis.com/git-repo-downloads/repo > /c/ti/repo
  echo "alias repo=\"winpty python /c/ti/repo\"" >> ~/.bashrc
  source ~/.bashrc
  ```

#### Cloning The Repositories

To clone the repositories, do below in your workarea folder:

1. Clone the Motor Control SDK repository

```bash
git clone https://github.com/TexasInstruments/motor-control-sdk.git motor_control_sdk
```

2. Clone the Industrial Communications SDK repository inside Motor Control SDK repository

```bash
cd motor_control_sdk
git clone https://github.com/TexasInstruments/ind-comms-sdk.git ind_comms_sdk
cd ..
```

3. Clone the MCU+ SDK component repositories inside Motor Control SDK repository using repo tool.

Note that depending on the SoC you're working with, the components you clone might be
slightly different. So please choose the manifest folder according to the SoC of your
interest. For example, we are showing for am243x below.

```bash
repo init -u https://github.com/TexasInstruments/mcupsdk-manifests.git -m am243x/dev.xml -b main
```

Note that repo uses symbolic links. So if you're on Windows and do not have permissions
to create symbolic links, the above command might fail for you. So you can either enable them
([refer this link](https://portal.perforce.com/s/article/3472)) or use the experimental
worktree feature of repo. To do this, initialize the repo like so:

```bash
repo init --worktree -u https://github.com/TexasInstruments/mcupsdk-manifests.git -m am243x/dev.xml -b main
```

After the repo is initialized, do a

```bash
repo sync
```

This should clone all the repositories required for Motor Control SDK development. Now download and install the dependencies.

#### Downloading And Installing Dependencies

**To download and install dependencies, follow the below steps**:

1. Download and install Code Composer Studio v12.6 from [here](https://www.ti.com/tool/download/CCSTUDIO "Code Composer Studio")
   - Install at default folder

2. Download and install SysConfig 1.19.0 from [here](https://www.ti.com/tool/download/SYSCONFIG "SYSCONFIG")
   - Install at default folder

3. Download and install Node.js v12.18.4 LTS
  - Go to the [NodeJS Website](https://nodejs.org/en/) and use the installer to
    download and install v12.18.4 of node. Install in the default directory.
  - After successful installation, run an `npm ci` inside the `motor_control_sdk` folder like so:
    ```bash
    $ cd motor_control_sdk/
    $ npm ci
    ```
    This should install the node packages required for the SDK.

4. Download and install doxygen,
   - Tested with 1.8.20
     - Download the correct version of doxygen from [here](https://www.doxygen.nl/download.html)
   - Test by doing below on the command prompt
     ```
     $ doxygen -v
     1.8.20 (<commit SHA-ID>)
     ```

**Installing OpenSSL**

Some of the SDK signing scripts are dependent on OpenSSL v1.1.1. The v1.1.1 is
important, OpenSSL 3.0 is not compatible with the current signing scripts of SDK.

In Windows,
  - Download v1.1.1 from [here](https://slproweb.com/products/Win32OpenSSL.html)
  - You can install the "light" version which is smaller download size
  - Install to default path, which is C:/Program Files/OpenSSL-Win64/
  - When prompted select option to install binaries to /bin folder of installed path instead of Windows system path.
  - Add path to OpenSSL, to your environment "Path" variable in windows
    - `C:\Program Files\OpenSSL-Win64\bin`

In Linux,
  - There is a chance that OpenSSL is already installed. If not, here are the steps:
  - If you have Ubuntu 18.04, do below in Linux Ubuntu shell to install openssl
    -`$ sudo apt install openssl`

    If you have an Ubuntu version higher than that, make sure that you install the 1.1.1 version.
    You can get the 1.1.1 packages from [here](http://security.ubuntu.com/ubuntu/pool/main/o/openssl/).
    The packages required are openssl, libssl and libssl-dev

Test openssl version by running `openssl version` on a command prompt and make sure there is no error.
Example output is shown below,

```bash
  C:\> openssl version
  OpenSSL 1.1.1k  25 Mar 2021
```

Now that the dependencies are installed, you can start the repositories with a
default branch `dev` by doing below:

```bash
repo start dev --all
```

---

**NOTE**

- In Linux, you will need to run `$HOME/ti/ccs{version}/ccs/install_scripts/install_drivers.sh` script for setting COM
  port accesses correctly. Also add your user to groups `tty` and `dialout`. You can do

  ```
  sudo adduser $USER tty
  sudo adduser $USER dialout
  ```

- Please use the industrial communication libraries from the sdk installer due licensing restrictions.

---

### Building the SDK

#### Basic Building With Makefiles

---

**NOTE**

- Use `gmake` in windows, add path to gmake present in CCS at `C:\ti\ccsxxxx\ccs\utils\bin` to your windows PATH. We have
  used `make` in below instructions.
- Unless mentioned otherwise, all below commands are invoked from root folder of the "motor_control_sdk"  repository.
- Current supported device names are am243x, and am263x
- Pass one of these values to `"DEVICE="`
- You can also build components (examples, tests or libraries) in `release` or `debug`
  profiles. To do this pass one of these values to `"PROFILE="`

---

1. Run the following command to create makefiles, this step is optional since this is invoked as part of other steps as well,

   ```bash
   make gen-buildfiles DEVICE=am243x
   ```

2. To see all granular build options, run

   ```bash
   make -s help DEVICE=am243x
   ```
   This should show you commands to build specific libraries, examples or tests.

3. Make sure to build the libraries before attempting to build an example. For example,
   to build a Tamagawa Diagnostic Single Channel example for AM243x, run the following:
   ```bash
   cd motor_control_sdk/mcu_plus_sdk
   make -s -j4 libs DEVICE=am243x PROFILE=debug
   cd ..
   make -s -j4 libs DEVICE=am243x PROFILE=debug
   ```
   Once the library build is complete, to build the example run:
   ```bash
   make -s -C examples/position_sense/tamagawa_diagnostic/single_channel/am243x-evm/r5fss0-0_freertos/ti-arm-clang all PROFILE=debug
   ```

4. Following are the commands to build **all libraries** and **all examples**. Valid PROFILE's are "release" or "debug"

   ```bash
   cd motor_control_sdk/mcu_plus_sdk
   make -s -j4 clean DEVICE=am243x PROFILE=debug
   make -s -j4 all   DEVICE=am243x PROFILE=debug
   cd ..
   make -s -j4 clean DEVICE=am243x PROFILE=debug
   make -s -j4 all   DEVICE=am243x PROFILE=debug
   ```