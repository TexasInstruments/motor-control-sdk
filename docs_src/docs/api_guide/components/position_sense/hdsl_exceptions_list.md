# TI HDSL Exceptions List {#HDSL_EXCEPTIONS_LIST}

Notable exceptions in TI HDSL Solution when compared with SICK HDSL IP Core release version 1.07 are described below:

1. SPI interface is not available to access the TI HDSL interface. Registers are present in Data Memory of Programmable Real-Time Unit and Industrial Communication Subsystem (PRU-ICSS), which can be accessed directly by the Arm®-based core.
2. Pipeline Status for SensorHub Channel is not available. Pipeline data is updated for each horizontal frame.
3. Control signals similar to SICK HDSL IP Core are not available, except SYNC signal. Instead of INTERRUPT signal, interrupts are triggered to Arm-based core.
4. Test signals similar to SICK HDSL IP Core are not available.
5. TI HDSL Solution's register map is register compatible with SICK HDSL IP Core release version 1.07, with a few exceptions listed below:

    <table>
    <tr>
        <th> Register(s)
        <th> Remarks
    </tr>
    <tr>
        <td> SYS_CTRL Bits 5:0 (FRST, LOOP, PRDY, SPPE, SPOL, OEN)
        <td> **Not available in TI HDSL Solution**
    </tr>
    <tr>
        <td> EVENT_H Bit 7 (INT)
        <td> **Not available in TI HDSL Solution**
    </tr>
   <tr>
        <td> EVENT_H Bit 1 (DTE) <br/>
             MASK_H Bit 1 (MDTE) <br/>
             ONLINE_STATUS_D_H Bit 1 (DTE)
        <td> **Not available in TI HDSL Solution**
    </tr>
    <tr>
        <td> EDGES
        <td> **Not available in TI HDSL Solution**<br/>
    </tr>
    <tr>
        <td> VERSION<br/>
            VERSION2
        <td> **Different implementation from SICK HDSL IP Core**<br/>
            "Major Release Number" field is 4 bits wide instead of 2 bits. "Coding" field is not available.
    </tr>
    <tr>
        <td> RELEASE
        <td> **Not available in TI HDSL Solution**
    </tr>
    <tr>
        <td> MIR_SUM
        <td> **Not available in TI HDSL Solution**<br/>
            Please see SAFE_SUM (0x36) for getting summary information.
    </tr>
    <tr>
        <td> PIPE_S<br/>
        <td> **Not available in TI HDSL Solution**
    </tr>
    <tr>
        <td> PC_DATA
        <td> **Not available in TI HDSL Solution**<br/>
            Please see S_PC_DATA (0x37) for "short message" transactions.
    </tr>
    <tr>
        <td> ACC_ERR_CNT
        <td> **Different implementation from SICK HDSL IP Core**<br/>
             - This register gives the count of transmitted fast position values with consecutive transmission errors.
             - Writing to this register does not set any threshold for setting an error signal. ACC_ERR_CNT_THRESH (0x41) register allows triggering protocol reset if ACC_ERR_CNT crosses a threshold.
             - This count is an 8 bit value.
    </tr>
    <tr>
        <td> MAXACC<br/>
             MAXDEV
        <td> **Not available in TI HDSL Solution**
    </tr>
    <tr>
        <td> ENC2_ID
        <td> **Not available in TI HDSL Solution**
    </tr>
    <tr>
        <td> EVENT_S Bit 7 (SINT)
        <td> **Not available in TI HDSL Solution**
    </tr>
    <tr>
        <td> POSTX<br/>
            ONLINE_STATUS_D_L Bits 7:6 (POSTX) <br/>
            ONLINE_STATUS_1_L Bits 7:6 (POSTX)<br/>
            ONLINE_STATUS_2_L Bits 7:6 (POSTX)
        <td> **Different implementation from SICK HDSL IP Core**<br/>
            POSTX bits are available in a separate POSTX register (0x4F) instead of ONLINE_STATUS_D_L, ONLINE_STATUS_1_L and ONLINE_STATUS_2_L registers.
    </tr>
    <tr>
        <td> ONLINE_STATUS_D_H Bit 7 (INT) <br/>
            ONLINE_STATUS_1_H Bit 7 (SINT)
        <td> **Not available in TI HDSL Solution**
    </tr>
    <tr>
        <td> ONLINE_STATUS_D_H Bit 6 (SUM)
        <td> **Different implementation from SICK HDSL IP Core** <br/>
             SAFE_SUM is used instead of MIR_SUM.
    </tr>
    <tr>
        <td> VERSION2<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x44 instead of 0x0B
    </tr>
    <tr>
        <td>
             ENC2_ID<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x45 instead of 0x0F
    </tr>
    <tr>
        <td>
             STATUS2<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x46 instead of 0x18
    </tr>
    <tr>
        <td>
             VPOS24<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x47 instead of 0x19
    </tr>
    <tr>
        <td>
             VPOS23<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x48 instead of 0x1A
    </tr>
    <tr>
        <td>
             VPOS22<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x49 instead of 0x1B
    </tr>
    <tr>
        <td>
             VPOS21<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x4A instead of 0x1C
    </tr>
    <tr>
        <td>
             VPOS20<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x4B instead of 0x1D
    </tr>
    <tr>
        <td>
             VPOSCRC2_H<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x4C instead of 0x1E
    </tr>
    <tr>
        <td>
             VPOSCRC2_L<br/>
        <td> **Register address is different from SICK HDSL IP Core** <br/>
             TI implementation uses 0x4D instead of 0x1F
    </tr>
    </table>

6. Reset values of registers are not the same as SICK HDSL IP Core.
7. As registers are implemented using Data Memory of Programmable Real-Time Unit and Industrial Communication Subsystem (PRU-ICSS), the application has read-write access for all registers.
8. When safe position is invalid (VPOS bit is set in EVENT_S), 0xFDFDFDFDFD value is not set in fast and safe position registers.
9. For long message offset, only 15-bit wide offset is supported. If offset is enabled, then the TI HDSL receiver will always send 2 bytes of offset.

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.
