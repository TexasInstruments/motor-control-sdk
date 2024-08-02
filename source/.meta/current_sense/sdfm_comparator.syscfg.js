function onChangeComparatorEnable(inst, ui)
{
    for (let channel = 0; channel < 9; channel++)
	{
		let status = inst["Ch" + channel.toString() + "_ComparatorEnable"];

		ui["Ch" + channel.toString() + "_OC_OSR"].hidden = !status;
		
		ui["Ch" + channel.toString() + "_Datarate_CF"].hidden = !status;
		ui["Ch" + channel.toString() + "_Latency_CF"].hidden = !status;

		ui["Ch" + channel.toString() + "_HLT"].hidden = !status;
		ui["Ch" + channel.toString() + "_LLT"].hidden = !status;

		ui["Ch" + channel.toString() + "_ZeroCross_Enable"].hidden = !status;

	}

}

function onChangeUseZeroCrossSettings(inst, ui)
{
    for (let channel = 0; channel < 9; channel++)
    {
        let status = inst["Ch" + channel.toString() + "_ZeroCross_Enable"];
        ui["Ch" + channel.toString() + "_ZCT"].hidden = !status;
    }
}
// Comparator settings //
function comparatorSettings(channel)
{
	let Settings = [];
	let order = 0;

	Settings =
    
	  [
		{
			name: "Ch" + channel.toString() + "_OC_OSR",
			displayName : "Over Current OSR",
			description : 'Over Current OSR',
			hidden      : true,
			default     : 16,
		},
		{
			name        : "Ch" + channel.toString() + "_Datarate_CF",
			displayName : "Data Rate (us)",
			description : 'Data rate (Data filter) for given OSR setting and SDmodulator rate',
			hidden      : true,
		    getValue    : (inst) => {
                let datarate = (inst["Ch" + channel.toString() + "_OC_OSR"]* 1000000 )/ inst["Ch" + channel.toString() + "_SD_modulatorFrequency"];
                return datarate;
            },
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Latency_CF",
			displayName : "Latency (us)",
			description : 'Latency (or) settling time (Data filter) for given OC OSR setting and SDmodulator rate',
			hidden      : true,
		    getValue    : (inst) => {
				if(inst["Ch" + channel.toString() + "_AccSource"] == "0")
				{
					order = 3;
				}
				if(inst["Ch" + channel.toString() + "_AccSource"] == "1")
				{
					order = 2;
				}
				if(inst["Ch" + channel.toString() + "_AccSource"] == "2")
				{
					order = 1;
				}
                let latency = order * (inst["Ch" + channel.toString() + "_OC_OSR"]* 1000000)/ inst["Ch" + channel.toString() + "_SD_modulatorFrequency"];
                return (latency + 0.3); /* 300ns Normal current Task */
            },
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_HLT",
			displayName : "High Level Threshold",
			description : 'High Level Threshold',
			hidden      : true,
            default     : "32767",
		},
		{
			name: "Ch" + channel.toString() + "_LLT",
            displayName : "Low Level Threshold",
            description : 'Low Level Threshold',
            hidden      : true,
            default     : "0",
		},
		{
			name: "Ch" + channel.toString() + "_ZeroCross_Enable",
			displayName : "Enable Zero Cross Detection",
			description : 'Enable Zero Cross Detection',
			hidden		: true,
			default     : false,
			onChange	: onChangeUseZeroCrossSettings,
		},
        {
            name: "GROUP_ZC_Settings" + channel.toString() +  "",
            displayName: "Zero Cross Settings",
            collapsed: false,
            config:
            [
                {
                    name        : "Ch" + channel.toString() + "_ZCT",
                    displayName : "Zero Cross Threshold",
                    description : 'Zero Cross Threshold',
                    hidden      : true,
                    default     : "32767",  
                },

          ]
        },
	]
    
    return(Settings);
}


exports =
{
    comparatorSettings : comparatorSettings,
	onChangeComparatorEnable : onChangeComparatorEnable,
}