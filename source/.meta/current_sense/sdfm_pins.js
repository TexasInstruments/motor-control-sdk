let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");


function getInterfaceName(inst)
{
    return `PRU_${inst.instance}_PRU`;
}

function getInterfacePinList(inst)
{
    let pinList = [];

    /*SDFM SD8_CLK clock*/
    pinList.push({ pinName: "GPI16", displayName: "SD_CLK8", rx: true});
    
    /*SD0_D*/
    pinList.push({ pinName: "GPI1", displayName: "SD_CHANNEL0_DATA", rx: true});

    /*SD1_D*/
    pinList.push({ pinName: "GPI3", displayName: "SD_CHANNEL1_DATA", rx: true});
    
    /*SD2_D*/
    pinList.push({ pinName: "GPI5", displayName: "SD_CHANNEL2_DATA", rx: true});

    /*SD3_D*/
    pinList.push({ pinName: "GPI7", displayName: "SD_CHANNEL3_DATA", rx: true});

    /*SD4_D*/
    pinList.push({ pinName: "GPI18", displayName: "SD_CHANNEL4_DATA", rx: true});

    /*SD5_D*/
    pinList.push({ pinName: "GPI11", displayName: "SD_CHANNEL5_DATA", rx: true});
   
    /*SD6_D*/
    pinList.push({ pinName: "GPI13", displayName: "SD_CHANNEL6_DATA", rx: true});

    /*SD7_D*/
    pinList.push({ pinName: "GPI15", displayName: "SD_CHANNEL7_DATA", rx: true});

    /*SD8_D*/
    pinList.push({ pinName: "GPI17", displayName: "SD_CHANNEL8_DATA", rx: true});

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

        if(inst["Channel_0"] == true){
            if((pin.pinName == "GPI1")){
                pinResource.used = true;
            }
         }else{
            if( (pin.pinName == "GPI1")){
                pinResource.used = false;
            }    
         }
         if(inst["Channel_1"] == true){
            if((pin.pinName == "GPI3")){
                pinResource.used = true;
            }
         }else{
            if( (pin.pinName == "GPI3")){
                pinResource.used = false;
            }    
         }
         if(inst["Channel_2"]==true){
            if((pin.pinName == "GPI5")){
                pinResource.used = true;
            }
         }else{
            if( (pin.pinName == "GPI5")){
                pinResource.used = false;
            }    
         }
         if(inst["Channel_3"]==true){
            if((pin.pinName == "GPI7")){
                pinResource.used = true;
            }
         }else{
            if( (pin.pinName == "GPI7")){
                pinResource.used = false;
            }    
         }
         if(inst["Channel_4"]==true){
            if((pin.pinName == "GPI18")){
                pinResource.used = true;
            }
         }else{
            if( (pin.pinName == "GPI18")){
                pinResource.used = false;
            }    
         }
         if(inst["Channel_5"]==true){
            if((pin.pinName == "GPI11")){
                pinResource.used = true;
            }
         }else{
            if( (pin.pinName == "GPI11")){
                pinResource.used = false;
            }    
         }
         if(inst["Channel_6"]==true){
            if((pin.pinName == "GPI13")){
                pinResource.used = true;
            }
         }else{
            if( (pin.pinName == "GPI13")){
                pinResource.used = false;
            }    
         }
         if(inst["Channel_7"]==true){
            if((pin.pinName == "GPI15")){
                pinResource.used = true;
            }
         }else{
            if( (pin.pinName == "GPI15")){
                pinResource.used = false;
            }    
         }
         if(inst["Channel_8"]==true){
            if((pin.pinName == "GPI17")){
                pinResource.used = true;
            }
         }else{
            if( (pin.pinName == "GPI17")){
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
