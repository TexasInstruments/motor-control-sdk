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
    pinList.push({ pinName: "GPI16", displayName: "SD8_CLK", rx: true});
    
    /*SD0_D*/
    pinList.push({ pinName: "GPI1", displayName: "SD_CHANNEL0_DATA", rx: true});

    /*SD1_D*/
    pinList.push({ pinName: "GPI3", displayName: "SD_CHANNEL1_DATA", rx: true});
    
    /*SD2_D*/
    pinList.push({ pinName: "GPI5", displayName: "SD_CHANNEL2_DATA", rx: true});

    /*SD3_D*/
    pinList.push({ pinName: "GPI7", displayName: "SD_CHANNEL3_DATA", rx: true});

    /*SD4_D*/
    pinList.push({ pinName: "GPI9", displayName: "SD_CHANNEL4_DATA", rx: true});

    /*SD5_D*/
    pinList.push({ pinName: "GPI11", displayName: "SD_CHANNEL5_DATA", rx: true});
   
    /*SD6_D*/
    pinList.push({ pinName: "GPI13", displayName: "SD_CHANNEL6_DATA", rx: true});

    /*SD7_D*/
    pinList.push({ pinName: "GPI15", displayName: "SD_CHANNEL7_DATA", rx: true});

    /*SD8_D*/
    pinList.push({ pinName: "GPI17", displayName: "SD_CHANNEL8_DATA", rx: true});

    /*SD0_CLK*/
    pinList.push({ pinName: "GPI0", displayName: "SD0_CLK", rx: true});
    /*SD1_CLK*/
    pinList.push({ pinName: "GPI2", displayName: "SD1_CLK", rx: true});
    /*SD2_CLK*/
    pinList.push({ pinName: "GPI4", displayName: "SD2_CLK", rx: true});
    /*SD3_CLK*/
    pinList.push({ pinName: "GPI6", displayName: "SD3_CLK", rx: true});
    /*SD4_CLK*/
    pinList.push({ pinName: "GPI8", displayName: "SD4_CLK", rx: true});
    /*SD5_CLK*/
    pinList.push({ pinName: "GPI10", displayName: "SD5_CLK", rx: true});
    /*SD6_CLK*/
    pinList.push({ pinName: "GPI12", displayName: "SD6_CLK", rx: true});
    /*SD7_CLK*/
    pinList.push({ pinName: "GPI14", displayName: "SD7_CLK", rx: true});
    /*GPO1 for SD clk */
    pinList.push({ pinName: "GPO1", displayName: "GPO1_CLK_OUT", rx: false});

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

        if(inst["Enable_Channel_0"] == true){
           if((pin.pinName == "GPI1")){
               pinResource.used = true;
           }
           if((pin.pinName == "GPO1")){
            pinResource.used = false;
        }
        }else{
           if( (pin.pinName == "GPI1")){
               pinResource.used = false;
           }    
        }
        if(inst["Enable_Channel_1"] == true){
           if((pin.pinName == "GPI3")){
               pinResource.used = true;
           }
           if((inst["Ch1_SDCLKSEL"]== "1")){
                if((pin.pinName == "GPI2")){
                    pinResource.used = true;
                }
           }
           else{
                if((pin.pinName == "GPI2")){
                        pinResource.used = false;
                }
           }
        }else{
           if( (pin.pinName == "GPI3")||(pin.pinName == "GPI2")){
               pinResource.used = false;
           }  
        }
        if(inst["Enable_Channel_2"]==true){
           if((pin.pinName == "GPI5")){
               pinResource.used = true;
           }
           if((inst["Ch2_SDCLKSEL"]== "1")){
                if((pin.pinName == "GPI4")){
                    pinResource.used = true;
                }
            }
            else{
                if((pin.pinName == "GPI4")){
                    pinResource.used = false;
                }
            }
        }else{
           if( (pin.pinName == "GPI5")||(pin.pinName == "GPI4")){
               pinResource.used = false;
            }  
        }
        if(inst["Enable_Channel_3"]==true){
           if((pin.pinName == "GPI7")){
               pinResource.used = true;
           }
        }else{
           if( (pin.pinName == "GPI7")){
               pinResource.used = false;
           }    
        }
        if(inst["Enable_Channel_4"]==true){
           if((pin.pinName == "GPI9")){
               pinResource.used = true;
           }
           if((inst["Ch4_SDCLKSEL"]== "1")){
                if((pin.pinName == "GPI8")){
                    pinResource.used = true;
                }
            }
            else{
                if((pin.pinName == "GPI8")){
                    pinResource.used = false;
                }
            }
        }else{
           if( (pin.pinName == "GPI9")||(pin.pinName == "GPI8")){
               pinResource.used = false;
            } 
        }
        if(inst["Enable_Channel_5"]==true){
           if((pin.pinName == "GPI11")){
               pinResource.used = true;
           }
           if((inst["Ch5_SDCLKSEL"]== "1")){
                if((pin.pinName == "GPI10")){
                    pinResource.used = true;
                }
            }
            else{
                if((pin.pinName == "GPI10")){
                    pinResource.used = false;
                }
            }
        }else{
           if( (pin.pinName == "GPI11") ||((pin.pinName == "GPI10"))){
               pinResource.used = false;
           } 
        }
        if(inst["Enable_Channel_6"]==true){
           if((pin.pinName == "GPI13")){
               pinResource.used = true;
           }
        }else{
           if( (pin.pinName == "GPI13")){
               pinResource.used = false;
           }    
        }
        if(inst["Enable_Channel_7"]==true){
           if((pin.pinName == "GPI15")){
               pinResource.used = true;
           }
           if((inst["Ch7_SDCLKSEL"]== "1")){
                if((pin.pinName == "GPI14")){
                    pinResource.used = true;
                }
            }
            else{
                if((pin.pinName == "GPI14")){
                    pinResource.used = false;
                }
            }
        }else{
           if( (pin.pinName == "GPI15")||(pin.pinName == "GPI14")){
               pinResource.used = false;
            }    
        }
        if(inst["Enable_Channel_8"]==true){
           if((pin.pinName == "GPI17")){
               pinResource.used = true;
           }
        }else{
           if((pin.pinName == "GPI17")){
               pinResource.used = false;
           }    
        }  
        if((inst["Ch0_SDCLKSEL"]== "1") ||(inst["Ch0_SDCLKSEL"]== "2")||((inst["Ch1_SDCLKSEL"]== "2"))||((inst["Ch2_SDCLKSEL"]== "2")))
        {
            if((pin.pinName == "GPI0"))
            {
                pinResource.used = true;
            }
        }
        else
        {
            if((pin.pinName == "GPI0"))
            {
                pinResource.used = false;
            }
        }  
        if((inst["Ch3_SDCLKSEL"]== "1") ||(inst["Ch3_SDCLKSEL"]== "2")||((inst["Ch4_SDCLKSEL"]== "2"))||((inst["Ch5_SDCLKSEL"]== "2")))
        {
            if((pin.pinName == "GPI6"))
            {
                pinResource.used = true;
            }
        }
        else
        {
            if((pin.pinName == "GPI6"))
            {
                pinResource.used = false;
            }
        }
        if((inst["Ch6_SDCLKSEL"]== "1") ||(inst["Ch6_SDCLKSEL"]== "2")||((inst["Ch7_SDCLKSEL"]== "2"))||((inst["Ch8_SDCLKSEL"]== "2")))
        {
            if((pin.pinName == "GPI12"))
            {
                pinResource.used = true;
            }
        }
        else
        {
            if((pin.pinName == "GPI12"))
            {
                pinResource.used = false;
            }
        }
        if((inst["SDFM_CLK_GEN"] == "1"))
        {
            if((pin.pinName == "GPI1"))
            {
                pinResource.used = false;
            }
            if(pin.pinName == "GPO1")
            {    
                pinResource.used = true;
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