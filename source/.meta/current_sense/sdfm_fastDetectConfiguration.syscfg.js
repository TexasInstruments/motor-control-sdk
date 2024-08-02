function onChangeFastDetectEnable(inst, ui)
{
    for (let channel = 0; channel < 9; channel++)
	{
		let status = inst["Ch" + channel.toString() + "_FastDetect"];

		ui["Ch" + channel.toString() + "_FD_SIZE"].hidden = !status;
		
		ui["Ch" + channel.toString() + "_ZeroMaxTh"].hidden = !status;
		ui["Ch" + channel.toString() + "_ZeroMinTh"].hidden = !status;

	}
}
function fastDetectConfigs(channel)
{
	let config =
		[
			{
				name: "Ch" + channel.toString() + "_FD_SIZE",
				displayName : "FD Window Size",
				description : 'FD Window Size',
				hidden      : true,
				default: "4",
                options: [
                    {
                        name: "0",
                        displayName: "4-bit"
                    },
                    {
                        name: "1",
                        displayName: "8-bit"
                    },
                    {
                        name: "2",
                        displayName: "12-bit"
                    },
                    {
                        name: "3",
                        displayName: "16-bit"
                    },
                    {
                        name: "4",
                        displayName: "20-bit"
                    },
                    {
                        name: "5",
                        displayName: "24-bit"
                    },
                    {
                        name: "6",
                        displayName: "28-bit"
                    },
                ],
			},
			{
				name: "Ch" + channel.toString() + "_ZeroMaxTh",
				displayName : "Zero Count Max Threshold",
				description : 'Zero Count Max Threshold',
				default: 18,
                hidden: true,
			},
            {
				name: "Ch" + channel.toString() + "_ZeroMinTh",
				displayName : "Zero Count Min Threshold",
				description : 'Zero Count Min Threshold',
				default: 18,
                hidden: true,
			},

        ]
	return(config);
}


exports = 
{
    fastDetectConfigs : fastDetectConfigs,
    onChangeFastDetectEnable : onChangeFastDetectEnable,
}