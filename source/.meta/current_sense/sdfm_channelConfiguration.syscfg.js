let useFastDetect = system.getScript("/current_sense/sdfm_fastDetectConfiguration.syscfg.js");
let useComparator = system.getScript("/current_sense/sdfm_comparator.syscfg.js");
let sd_definition = system.getScript("/current_sense/sdfm_math.syscfg.js")
let sdfm_clk_config = system.getScript("/current_sense/sdfm_clockConfiguration.syscfg.js");


let density_1s = 0;
let Theoretical_Data_Filter = 0;
let Theoretical_Comparator_Filter = 0;

function SDmodulatorSettings(channel)
{
	let Settings = [];

	density_1s = (inst) => {
                let temp= (inst["Ch" + channel.toString() + "_DC_Input"] + inst["Ch" + channel.toString() + "_Vclipping"]) / (2 * inst["Ch" + channel.toString() + "_Vclipping"]);
                return temp;}
				
	Theoretical_Data_Filter = (inst) => {
					let temp = Math.floor((Math.abs(inst["Ch" + channel.toString() + "_DC_Input"]) / inst["Ch" + channel.toString() + "_Vclipping"])*sd_definition.getFilterOutputRange(inst["Ch" + channel.toString() + "_AccSource"], inst["Ch" + channel.toString() + "_NC_OSR"]));
					if(inst["Ch" + channel.toString() + "_DC_Input"] < 0)
					{
						temp = -1 * temp;
					}
					return temp;}
	
	Theoretical_Comparator_Filter = (inst) => {
					let temp = (inst["Ch" + channel.toString() + "_DC_Input"] + inst["Ch" + channel.toString() + "_Vclipping"]) / (2 * inst["Ch" + channel.toString() + "_Vclipping"]) * sd_definition.getFilterOutputRange(inst["Ch" + channel.toString() + "_AccSource"], inst["Ch" + channel.toString() + "_OC_OSR"]);
					return temp;}
	

	Settings =
	[
		{
			name: "Ch" + channel.toString() + "_SD_modulatorFrequency",
			displayName : "SD Modulator Frequency (Hz)",
			description : 'Select SD Modulator Frequency in Hz',
			hidden      : true,
			getValue    : (inst) => {
                let SDMCLK =  inst["Ch" + channel.toString() + "_SDFM Clock"];
                return SDMCLK;
            },
            default     : 20000000,
		},
		{
			name        : "Ch" + channel.toString() + "_Vclipping",
			displayName : "Differential Clipping Voltage (V)",
			description : 'What is the full scale range of SD modulator',
			hidden      : true,
			default     : 0.32,
		},
		{
			name        : "Ch" + channel.toString() + "_DC_Input",
			displayName : "DC Input To SD-modulator (V)",
			description : 'What is the DC input to SD-modulator',
			hidden      : true,
			default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_bitstream_1s_density",
			displayName : "Bitstream 1's Density",
			description : 'Density of 1s in SD modulated bitstream',
			hidden      : true,
 		    getValue    : density_1s,
            default     : 0,
		},

		{
			name        : "Ch" + channel.toString() + "_Theoritical_DataFilterOutput",
			displayName : "Theoritical Data Filter Output",
			description : 'Theoritical Data filter Output',
			hidden      : true,
			getValue    : Theoretical_Data_Filter,
		    default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Theoritical_ComparatorFilterOutput",
			displayName : "Theoritical Comparator Filter Output",
			description : 'Theoritical Comparator filter Output',
			hidden      : true,
			getValue    : Theoretical_Comparator_Filter,
		    default     : 0,
		},

	
	]
	return(Settings);
}
// SD Normal current settings //

function configNCsamplingMode(inst, ui)
{
    for (let channel = 0; channel < 9; channel++)
	{
        let status = inst["Ch" + channel.toString() + "_EnableContinuousMode"];
		let status1 = inst["Enable_Channel_" + channel.toString()];
        
        if(status1)
		{
			ui["Ch" + channel.toString() + "_FirstTriggerPoint"].hidden = status;
			ui["Ch" + channel.toString() + "_EnableDoubleUpdate"].hidden = status;
		}
		
	}

}

function doubleUpdateConfig(inst, ui)
{
    for (let channel = 0; channel < 9; channel++)
	{
        let status = inst["Ch" + channel.toString() + "_EnableDoubleUpdate"];
        ui["Ch" + channel.toString() + "_SecondTriggerPoint"].hidden = !status;
 
	}

}

function onChangeEnableEPWM(inst, ui)
{
	for (let channel = 0; channel < 9; channel++)
    {
        let status = inst["Ch" + channel.toString() + "_EPWM_SYNC"];
        ui["Ch" + channel.toString() + "_Epwm_Source"].hidden = !status;
    }
}

function epwmSettings(channel)
{
	let Settings = [];
	
	Settings =
    [
		{
			name: "Ch" + channel.toString() + "_Epwm_Source",
			displayName : "Source of SD SYNC Event",
			description : 'Source of SD SYNC Event',
			hidden      : true,
			default     : "EPWM0",
			options     :
			[
				{
					name: "EPWM0",
					displayName: "SDFM SYNC source is EPWM0 SYNC out event",
				},
				{
					name:"EPWM3",
					displayName: "SDFM SYNC source is EPWM3 SYNC out event",
				},
			]
		},
	]
	return(Settings)
}
function SDnormalCurrentConfigs(channel)
{
	let Settings = [];

	Settings =
	[
		{
			name: "Ch" + channel.toString() + "_NC_OSR",
			displayName : "Normal Current OSR",
			description : 'Normal Current OSR',
			hidden      : true,
			default     : 64,
		},
		{
			name        : "Ch" + channel.toString() + "_EnableContinuousMode",
			displayName : "Enable NC Continuous Mode",
			description : 'Enable NC Continuous Mode',
			hidden      : true,
            default     : false,
            onChange	: configNCsamplingMode,        
		},
		{
			name        : "Ch" + channel.toString() + "_FirstTriggerPoint",
			displayName : "First Trigger Point (us)",
			description : 'First Trigger Point (us)',
			hidden      : true,
			default     : 15,
		},
        {
			name        : "Ch" + channel.toString() + "_EnableDoubleUpdate",
			displayName : "Enable Double Update",
			description : 'Enable Double Update',
			hidden      : true,
            default     : false,
		    onChange    : doubleUpdateConfig,
		},
        {
			name        : "Ch" + channel.toString() + "_SecondTriggerPoint",
			displayName : "Second Trigger Point (us)",
			description : 'Second Trigger Point (us)',
			hidden      : true,
			default     : 30,
		},
        {
			name        : "Ch" + channel.toString() + "_Datarate_DF",
			displayName : "Data Rate (us)",
			description : 'Data rate (Data filter) for given OSR setting and SDmodulator rate',
			hidden      : true,
		    getValue    : (inst) => {
                let datarate = (inst["Ch" + channel.toString() + "_NC_OSR"]*1000000) / inst["Ch" + channel.toString() + "_SD_modulatorFrequency"];
                return datarate;
            },
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Latency_DF",
			displayName : "Latency (us)",
			description : 'Latency (or) settling time (Data filter) for given OSR setting and SDmodulator rate',
			hidden      : true,
		    getValue    : (inst) => {
				let order ;
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
                let latency = (order * (inst["Ch" + channel.toString() + "_NC_OSR"] * 1000000))/ inst["Ch" + channel.toString() + "_SD_modulatorFrequency"];
                return latency;
            },
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Min_FilterOutput",
			displayName : "Min (Normal Current Output)",
			description : 'Minimum Data Filter output for given NC OSR setting',
			hidden      : true,
		    getValue    : (inst) => {
                let min_value = -1*sd_definition.getFilterOutputRange(inst["Ch" + channel.toString() + "_AccSource"], inst["Ch" + channel.toString() + "_NC_OSR"]);
                return min_value;
            },
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Max_FilterOutput",
			displayName : "Max (Normal Current Output)",
			description : 'Maximum Data Filter output for given OSR setting',
			hidden      : true,
		   getValue    : (inst) => {
                let max_value = sd_definition.getFilterOutputRange(inst["Ch" + channel.toString() + "_AccSource"], inst["Ch" + channel.toString() + "_NC_OSR"]);
				return max_value;
            },
            default     : 0,
		},
		{
			name: "Ch" + channel.toString() + "_EPWM_SYNC",
			displayName : "Use EPWM Synchronization",
			description : 'Use EPWM Synchronization',
			hidden      : true,
			default     : false,
			onChange    : onChangeEnableEPWM
		},
		{
			name        : "GROUP_EPWM",
			displayName : "SDSYNC Feature Settings",
			config      : epwmSettings(channel)
		},

	]
	return(Settings);
}

function fill_channel_array(channel)
{
	let config =
		[
            {
				name: "Ch" + channel.toString() + "_AccSource",
				displayName : "SD Accumulator Source",
				description : 'SD Accumulator Source',
				hidden      : true,
				default     : "0",
                options: [
                    {
                        name: "0",
                        displayName: "SINC3"
                    },
                    {
                        name: "1",
                        displayName: "SINC2"
                    },
                    {
                        name: "2",
                        displayName: "SINC1"
                    },
                ],
			},
            {
                name: "Ch" + channel.toString() + "_ComparatorEnable",
                displayName : "Enable Comparator",
                description : 'Enable Comparator',
                hidden      : true,
                default     : false,
                onChange    : useComparator.onChangeComparatorEnable
            },
            {
				name        : "GROUP_ComparatorFilter",
				displayName : "Comparator Settings",
				config      : useComparator.comparatorSettings(channel)
			},
            {
				name: "Ch" + channel.toString() + "_FastDetect",
				displayName : "Enable Fast Detect",
				description : 'Enable / Disable Fast Detect',
				hidden      : true,
				default     : false,
				onChange	: useFastDetect.onChangeFastDetectEnable
			},

			{
				name        : "GROUP_FastDetect",
				displayName : "Fast Detect Configuration",
				config      : useFastDetect.fastDetectConfigs(channel)
			},
		    
            {
				name: "GROUP_NormalCurrent",
				displayName : "Normal Current Configurtion",
				collapsed   : false,
				config     : SDnormalCurrentConfigs(channel),
			},
			{
				name        : "GROUP_SDMod",
				displayName : "SD_modulator_Settings",
				collapsed	: false,
				config      : SDmodulatorSettings(channel),
			},

        ]
	return(config);
}


let channelConfigs = [];

for (let channel = 0; channel < 9; channel++)
{
	channelConfigs = channelConfigs.concat
	(
	  [
	   	{
       	 name: "GROUP_CHANNEL" + channel.toString(),
       	 displayName: "Channel" + channel.toString(),
       	 description: "Configure Channel" + channel.toString(),
			collapsed	: true,
       	 longDescription: "",
       	 config: fill_channel_array(channel),
	   	}
	  ]
	);
}


let sdfmChannelConfig = {
    displayName: "SDFM Channel Configuration",
    defaultInstanceName: "SDFM_ChannelConfig",
    description: "SDFM Channel Configuration",
    config: channelConfigs,
    templates: {
        boardc : "",
        boardh : ""
    },
};

exports = sdfmChannelConfig;