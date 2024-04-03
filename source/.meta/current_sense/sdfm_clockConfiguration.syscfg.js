let common   = system.getScript("/common");
 
let SDFM_ClockSource = [
	{ name: "SDFM_CLK_SOURCE_SD8_CLK", displayName: "Source is SD8_CLK common for all nine channels" },
	{ name: "SDFM_CLK_SOURCE_SDi_CLK", displayName: "Source is respective channel clock" },
    { name: "SDFM_CLK_SOURCE_SDXi_CLK", displayName: "Source is common clock for group of 3 channels (SD0_CLK for 0-2, SD3_CLK for 3-5, SD6_CLK for 6-8" },
]

let ClockConfigs = [];

for (let channel = 0; channel < 9; channel++)
{
	ClockConfigs = ClockConfigs.concat
	(
	  [
	    {
         name: "Ch" + channel.toString() + "_SDCLKSEL",
         displayName: "Channel" + channel.toString() + " SDCLK Source",
         description: "Channel" + channel.toString() + " SDCLK Source",
		    hidden	: true,
         default: SDFM_ClockSource[0].name,
         options: SDFM_ClockSource
	    },
        {
         name: "Ch" + channel.toString() + "_SDFM Clock",
         displayName: "Channel" + channel.toString() + " SDFM Clock (Hz)",
         description: "Channel" + channel.toString() + " SDFM Clock (Hz)",
		    hidden	: true,
         default: 20000000,
        },
        {
         name: "Ch" + channel.toString() + "_CLKINV",
         displayName: "Enable Channel" + channel.toString() + " Clock Inversion",
         description: "Enable Channel" + channel.toString() + " Clock Inversion",
		    hidden: true,
         default: false,
	    },  
	  ]
	);
}

let sdfmClockConfiguration = {
    displayName: "SDFM Clock Configuration",
    defaultInstanceName: "SDFM_ClockConfig",
    description: 'SDFM Clock Configuration',
    config: ClockConfigs,
    templates: {
        boardc : "",
        boardh : ""
    },
};

exports = sdfmClockConfiguration