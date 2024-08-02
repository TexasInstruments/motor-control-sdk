
let common = system.getScript("/common");
let device = common.getDeviceName();
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let is_am263x_soc = (device === "am263x-cc") ? true : false;
let is_am261x_soc = (device === "am261x-lp") ? true : false;

function getInterfaceName(inst)
{
    if(is_am263x_soc) {
        return inst.instance;
    } else if (is_am261x_soc) {
        if(inst.instance === "ICSSM0") {
            return "PRU-ICSS0";
        } else {
            return "PRU-ICSS1";
        }
    }
}

function getInterfacePinList(inst)
{
    let pinList = [];

    let Slice = inst["PRU_Slice"];

    if (is_am261x_soc) {
        if(inst.instance === "ICSSM0") {
            Slice = "PR0_"+Slice;
        } else {
            Slice = "PR1_"+Slice;
        }
    } else {
        Slice = "PR0_"+Slice;
    }

    pinList.push({ pinName: Slice.toString()+"_GPIO2", displayName: "TAMAGAWA_CHANNEL0_TX_ENABLE", rx: false});
    pinList.push({ pinName: Slice.toString()+"_GPIO1", displayName: "TAMAGAWA_CHANNEL0_TX", rx: false});
    pinList.push({ pinName: Slice.toString()+"_GPIO0", displayName: "TAMAGAWA_CHANNEL0_CLK", rx: false});
    pinList.push({ pinName: Slice.toString()+"_GPIO9", displayName: "TAMAGAWA_CHANNEL0_RX", rx: true});

    pinList.push({ pinName: Slice.toString()+"_GPIO5", displayName: "TAMAGAWA_CHANNEL1_TX_ENABLE", rx: false});
    pinList.push({ pinName: Slice.toString()+"_GPIO4", displayName: "TAMAGAWA_CHANNEL1_TX", rx: false});
    pinList.push({ pinName: Slice.toString()+"_GPIO3", displayName: "TAMAGAWA_CHANNEL1_CLK", rx: false});
    pinList.push({ pinName: Slice.toString()+"_GPIO10", displayName: "TAMAGAWA_CHANNEL1_RX", rx: true});

    pinList.push({ pinName: Slice.toString()+"_GPIO8", displayName: "TAMAGAWA_CHANNEL2_TX_ENABLE", rx: false});
    /* PR0_PRU0_GPIO7 is not pinned out at the device level and therefore not supported for PRU0 */
    if(Slice == "PR0_PRU1")
    {
      pinList.push({ pinName: Slice.toString()+"_GPIO7", displayName: "TAMAGAWA_CHANNEL2_TX", rx: false});
    }
    pinList.push({ pinName: Slice.toString()+"_GPIO6", displayName: "TAMAGAWA_CHANNEL2_CLK", rx: false});
    pinList.push({ pinName: Slice.toString()+"_GPIO11", displayName: "TAMAGAWA_CHANNEL2_RX", rx: true});

    return pinList;
}

function pinmuxRequirements(inst) {

    let interfaceName = getInterfaceName(inst);
    let pinList = getInterfacePinList(inst);
    let resources = [];

    let Slice = inst["PRU_Slice"];

    if (is_am261x_soc) {
        if(inst.instance === "ICSSM0") {
            Slice = "PR0_"+Slice;
        } else {
            Slice = "PR1_"+Slice;
        }
    } else {
        Slice = "PR0_"+Slice;
    }

    for(let pin of pinList)
    {
        let pinResource = pinmux.getPinRequirements(interfaceName, pin.pinName, pin.displayName);

        pinmux.setConfigurableDefault( pinResource, "rx", pin.rx );

        if(inst["channel_0"]==true){
            if( (pin.pinName == Slice.toString()+"_GPIO2") || (pin.pinName == Slice.toString()+"_GPIO1") || (pin.pinName == Slice.toString()+"_GPIO0") || (pin.pinName == Slice.toString()+"_GPIO9")){
                 pinResource.used = true;
            }
        }else{
            if( (pin.pinName == Slice.toString()+"_GPIO2") || (pin.pinName == Slice.toString()+"_GPIO1") || (pin.pinName == Slice.toString()+"_GPIO0") || (pin.pinName == Slice.toString()+"_GPIO9")){
                pinResource.used = false;
            }
        }

        if(inst["channel_1"]==true){
            if( (pin.pinName == Slice.toString()+"_GPIO5") || (pin.pinName == Slice.toString()+"_GPIO4") || (pin.pinName == Slice.toString()+"_GPIO3") || (pin.pinName == Slice.toString()+"_GPIO10")){
                  pinResource.used = true;
             }
        }else{
            if( (pin.pinName == Slice.toString()+"_GPIO5") || (pin.pinName == Slice.toString()+"_GPIO4") || (pin.pinName == Slice.toString()+"_GPIO3") || (pin.pinName == Slice.toString()+"_GPIO10")){
                pinResource.used = false;
            }
        }

        if(Slice =="PR0_PRU1")
        {
            if(inst["channel_2"]==true){
                if( (pin.pinName == Slice.toString()+"_GPIO8") || (pin.pinName == Slice.toString()+"_GPIO7") || (pin.pinName == Slice.toString()+"_GPIO6") || (pin.pinName == Slice.toString()+"_GPIO11")){
                    pinResource.used = true;
                }
            }else{
                    if( (pin.pinName == Slice.toString()+"_GPIO8") || (pin.pinName == Slice.toString()+"_GPIO7") || (pin.pinName == Slice.toString()+"_GPIO6") || (pin.pinName == Slice.toString()+"_GPIO11")){
                    pinResource.used = false;
                }
            }
        }else{
            if(inst["channel_2"]==true){
                if( (pin.pinName == Slice.toString()+"_GPIO8") || (pin.pinName == Slice.toString()+"_GPIO6") || (pin.pinName == Slice.toString()+"_GPIO11")){
                    pinResource.used = true;
                }
            }else{
                    if( (pin.pinName == Slice.toString()+"_GPIO8") || (pin.pinName == Slice.toString()+"_GPIO6") || (pin.pinName == Slice.toString()+"_GPIO11")){
                    pinResource.used = false;
                }
            }
        }


        resources.push( pinResource );
    }

    let peripheralRequirements = {
        name: interfaceName,
        displayName: interfaceName,
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheralRequirements];
}

function getPeripheralPinNames(inst)
{
    let pinList = [];
    let pinNameList = [];

    pinList = getInterfacePinList(inst);

    for(let pin of pinList)
    {
        pinNameList.push( pin.pinName );
    }

    return pinNameList;
}


exports = {

    pinmuxRequirements,
    getInterfaceName,
    getPeripheralPinNames,
};
