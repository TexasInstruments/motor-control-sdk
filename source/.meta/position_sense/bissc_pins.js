
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");


function getInterfaceName(inst)
{
    return `PRU_${inst.instance}_PRU`;
}

function getInterfacePinList(inst)
{
    let pinList = [];

    /* BISSC_CHANNEL0_TX_ENABLE - is configured as soc gpio and kept low forever */
    /*pinList.push({ pinName: "GPO2", displayName: "BISSC_CHANNEL0_TX_ENABLE", rx: false});*/
    pinList.push({ pinName: "GPO1", displayName: "BISSC_CHANNEL0_TX", rx: false});
    pinList.push({ pinName: "GPO0", displayName: "BISSC_CHANNEL0_CLK", rx: false});
    pinList.push({ pinName: "GPI13", displayName: "BISSC_CHANNEL0_RX", rx: true});

    /* BISSC_CHANNEL1_TX_ENABLE - is configured as soc gpio and kept low forever */
    /*pinList.push({ pinName: "GPO5", displayName: "BISSC_CHANNEL1_TX_ENABLE", rx: false});*/
    pinList.push({ pinName: "GPO4", displayName: "BISSC_CHANNEL1_TX", rx: false});
    pinList.push({ pinName: "GPO3", displayName: "BISSC_CHANNEL1_CLK", rx: false});
    pinList.push({ pinName: "GPI14", displayName: "BISSC_CHANNEL1_RX", rx: true});

    /* BISSC_CHANNEL2_TX_ENABLE - is configured as soc gpio and kept low forever */
    /*pinList.push({ pinName: "GPO8", displayName: "BISSC_CHANNEL2_TX_ENABLE", rx: false});*/
    pinList.push({ pinName: "GPO12", displayName: "BISSC_CHANNEL2_TX", rx: false});
    pinList.push({ pinName: "GPO6", displayName: "BISSC_CHANNEL2_CLK", rx: false});
    pinList.push({ pinName: "GPI11", displayName: "BISSC_CHANNEL2_RX", rx: true});

    return pinList;
}

function pinmuxRequirements(inst) {

    let interfaceName = getInterfaceName(inst);
    let pinList = getInterfacePinList(inst);
    let resources = [];

    for(let pin of pinList)
    {
        let pinResource = pinmux.getPinRequirements(interfaceName, pin.pinName, pin.displayName);

        pinmux.setConfigurableDefault( pinResource, "rx", pin.rx );

        if(inst["channel_0"]==true){
            if( (pin.pinName == "GPO1") || (pin.pinName == "GPO0") || (pin.pinName == "GPI13")){
                 pinResource.used = true;
            }
        }else{
            if( (pin.pinName == "GPO1") || (pin.pinName == "GPO0") || (pin.pinName == "GPI13")){
                pinResource.used = false;
            }    
        }
 
        if(inst["channel_1"]==true){
             if( (pin.pinName == "GPO4") || (pin.pinName == "GPO3") || (pin.pinName == "GPI14")){
                  pinResource.used = true;
             }
        }else{
            if( (pin.pinName == "GPO4") || (pin.pinName == "GPO3") || (pin.pinName == "GPI14")){
                pinResource.used = false;
            }    
        }
 
        if(inst["channel_2"]==true){
            if( (pin.pinName == "GPO12") || (pin.pinName == "GPO6") || (pin.pinName == "GPI11")){
                pinResource.used = true;
            }
        }else{
            if( (pin.pinName == "GPO12") || (pin.pinName == "GPO6") || (pin.pinName == "GPI11")){
                pinResource.used = false;
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
