let inst_name;


function onValidate(inst, validation)
{
  for (let channel = 0; channel < 9; channel++)
  {
    inst_name= "Ch" + channel.toString() + "_NC_OSR";
	let nosr = inst[inst_name];

	if(nosr < 4 || nosr > 256)
	{
		validation.logError(
            "NC OSR should be between 4 to 256! - Check Channel_" + channel + " NC OSR = " + nosr,
            inst, inst_name);
	}
    inst_name= "Ch" + channel.toString() + "_OC_OSR";
	let cosr = inst[inst_name];

	if(cosr < 8 || cosr > 256)
	{
		validation.logError(
            "OC OSR should be between 4 to 256! - Check Channel_" + channel + " OC OSR = " + cosr,
            inst, inst_name);
	}

    inst_name= "Ch" + channel.toString() + "_HLT";
	let HLT = inst[inst_name];

	if(HLT < 0 || HLT > 16777216)
	{
		validation.logError(
            "HLT1 should be between 0 to 16777216! - Check Channel_" + channel + " HLT = " + HLT,
            inst, inst_name);
	}
    
    
    inst_name= "Ch" + channel.toString() + "_LLT";
	let LLT = inst[inst_name];

	if(LLT < 0 || LLT > 16777216)
	{
		validation.logError(
            "LLT should be between 0 to 16777216! - Check Channel_" + channel + " LLT = " + LLT,
            inst, inst_name);
	}

  }

  inst_name = "Enable_Phase_Compensation";
  let Phase_Delay = inst[inst_name];
  inst_name = "Enable_Load_Share";
  let Load_Share = inst[inst_name];
  
  if(Load_Share && Phase_Delay)
  {
    validation.logWarning(
        "Load Share sdk examples do not support Phase Compensation",
        inst, inst_name);
  }

  inst_name = "SDFM_CLK_GEN";
  let clock_gen = inst[inst_name];

  if(clock_gen == "3" && (!Phase_Delay))
  {
    validation.logWarning(
        "In sdk example, SDFM clock generation from IEP0 is only supported with Phase Compensation",
        inst, inst_name);
  }
  if(((clock_gen != "3") && Phase_Delay))
  {
    validation.logError(
        "Select SDFM clock generation from IEP0 ",
        inst, inst_name);

  }

  let channel_id = 0;
  inst_name = "Enable_Channel_0";
  let ch0 = inst[inst_name];
  inst_name = "Enable_Channel_1";
  let ch1 = inst[inst_name];
  inst_name= "Enable_Channel_2";
  let ch2 = inst[inst_name];
  channel_id = (ch0|(ch1<<1)|(ch2<<2));

  inst_name= "Ch0_SDFM Clock";
  let clock_ch0 = inst[inst_name];
  inst_name= "Ch1_SDFM Clock";
  let clock_ch1 = inst[inst_name];
  inst_name= "Ch2_SDFM Clock";
  let clock_ch2 = inst[inst_name];

  inst_name= "Ch0_NC_OSR";
  let nosr_ch0 = inst[inst_name];
  inst_name= "Ch1_NC_OSR";
  let nosr_ch1 = inst[inst_name];
  inst_name= "Ch2_NC_OSR";
  let nosr_ch2 = inst[inst_name];

  inst_name= "Ch0_EnableDoubleUpdate";
  let double_update_ch0 = inst[inst_name];
  inst_name= "Ch1_EnableDoubleUpdate";
  let double_update_ch1 = inst[inst_name];
  inst_name= "Ch2_EnableDoubleUpdate";
  let double_update_ch2 = inst[inst_name];

  inst_name= "Ch0_EnableContinuousMode";
  let cont_mode_ch0 = inst[inst_name];
  inst_name= "Ch1_EnableContinuousMode";
  let cont_mode_ch1 = inst[inst_name];
  inst_name= "Ch2_EnableContinuousMode";
  let cont_mode_ch2 = inst[inst_name];

  inst_name= "Ch0_FirstTriggerPoint";
  let fist_trig_point_ch0 = inst[inst_name];
  inst_name= "Ch1_FirstTriggerPoint";
  let fist_trig_point_ch1 = inst[inst_name];
  inst_name= "Ch2_FirstTriggerPoint";
  let fist_trig_point_ch2 = inst[inst_name];

  inst_name= "Ch0_SecondTriggerPoint";
  let second_trig_point_ch0= inst[inst_name];
  inst_name= "Ch1_SecondTriggerPoint";
  let second_trig_point_ch1 = inst[inst_name];
  inst_name= "Ch2_SecondTriggerPoint";
  let second_trig_point_ch2 = inst[inst_name];
  
  inst_name= "Ch0_EPWM_SYNC";
  let epwm_sync_ch0= inst[inst_name];
  inst_name= "Ch1_EPWM_SYNC";
  let epwm_sync_ch1 = inst[inst_name];
  inst_name= "Ch2_EPWM_SYNC";
  let epwm_sync_ch2 = inst[inst_name];

  inst_name= "Ch0_Epwm_Source";
  let epwm_source_ch0 = inst[inst_name];
  inst_name= "Ch1_Epwm_Source";
  let epwm_source_ch1 = inst[inst_name];
  inst_name= "Ch2_Epwm_Source";
  let epwm_source_ch2 = inst[inst_name];
  
  switch (channel_id)
  {
      
    case 3:
        if(clock_ch0 != clock_ch1 )
        {
            inst_name= "Ch0_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel0 and Channel1",
                inst, inst_name);
        }
        if(nosr_ch0 != nosr_ch1 )
        {
            inst_name= "Ch0_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel0 and Channel1",
                inst, inst_name);
        }
        if(double_update_ch0!=double_update_ch1)
        {
            inst_name= "Ch0_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel0 and Channel1",
                inst, inst_name);
        }
        if(cont_mode_ch0!=cont_mode_ch1)
        {
            inst_name= "Ch0_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel0 and Channel1",
                inst, inst_name);
        }
        if(fist_trig_point_ch0!=fist_trig_point_ch1)
        {
            inst_name= "Ch0_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        if(second_trig_point_ch0!=second_trig_point_ch1)
        {
            inst_name= "Ch0_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        if(epwm_sync_ch0!=epwm_sync_ch1)
        {
            inst_name= "Ch0_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel0 and Channel1",
                inst, inst_name);
        }
        if(epwm_source_ch0!=epwm_source_ch1)
        {
            inst_name= "Ch0_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        break;
    case 5:
        if(clock_ch0 != clock_ch2 )
        {
            inst_name= "Ch0_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel0 and Channel2",
                inst, inst_name);  
        }
        if(nosr_ch0 != nosr_ch2 )
        {
            inst_name= "Ch0_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel0 and Channel2",
                inst, inst_name);  
        }
        if(double_update_ch0!=double_update_ch2)
        {
            inst_name= "Ch0_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel0 and Channel2",
                inst, inst_name);
        }
        if(cont_mode_ch0!=cont_mode_ch2)
        {
            inst_name= "Ch0_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel0 and Channel2",
                inst, inst_name);
        }
        if(fist_trig_point_ch0!=fist_trig_point_ch2)
        {
            inst_name= "Ch0_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        } 
        if(second_trig_point_ch0!=second_trig_point_ch2)
        {
            inst_name= "Ch0_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }    
        if(epwm_sync_ch0!=epwm_sync_ch2)
        {
            inst_name= "Ch0_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel0 and Channel2",
                inst, inst_name);
        } 
        if(epwm_source_ch0!=epwm_source_ch2)
        {
            inst_name= "Ch0_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        break;
    case 6:
        if(clock_ch1 != clock_ch2 )
        {
            inst_name= "Ch1_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel1 and Channel2",
                inst, inst_name);
        }
        
        if(nosr_ch1 != nosr_ch2 )
        {
            inst_name= "Ch1_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel1 and Channel2",
                inst, inst_name);
        }

        if(double_update_ch2!=double_update_ch1)
        {
            inst_name= "Ch1_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel1 and Channel2 ",
                inst, inst_name);
        }
        if(cont_mode_ch2!=cont_mode_ch1)
        {
            inst_name= "Ch1_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel1 and Channel2",
                inst, inst_name);
        }
        if(fist_trig_point_ch2!=fist_trig_point_ch1)
        {
            inst_name= "Ch1_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        if(second_trig_point_ch2!=second_trig_point_ch1)
        {
            inst_name= "Ch2_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        if(epwm_sync_ch2!=epwm_sync_ch1)
        {
            inst_name= "Ch1_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel1 and Channel2",
                inst, inst_name);
        }
        if(epwm_source_ch2!=epwm_source_ch1)
        {
            inst_name= "Ch1_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        break;
    case 7:
        if((clock_ch0 != clock_ch1 )||(clock_ch0 != clock_ch2)||(clock_ch1!=clock_ch2))
        {
            inst_name= "Ch0_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel0-2",
                inst, inst_name);
        }
        if((nosr_ch0 != nosr_ch1)||(nosr_ch1 != nosr_ch2 )||(nosr_ch0 != nosr_ch2))
        {
            inst_name= "Ch0_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel0 - 2",
                inst, inst_name);
        }
        if((double_update_ch2!=double_update_ch1)||(double_update_ch2!=double_update_ch0)||(double_update_ch0!=double_update_ch1))
        {
            inst_name= "Ch0_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel0, Channel1 and Channel2 ",
                inst, inst_name);
        }
        if((cont_mode_ch2!=cont_mode_ch1)||(cont_mode_ch0!=cont_mode_ch1)||(cont_mode_ch2!=cont_mode_ch0))
        {
            inst_name= "Ch0_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel0, Channel1 and Channel2",
                inst, inst_name);  
        }
        if((fist_trig_point_ch0!=fist_trig_point_ch1)||(fist_trig_point_ch2!=fist_trig_point_ch1)||(fist_trig_point_ch0!=fist_trig_point_ch2))
        {
            inst_name= "Ch0_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        if((second_trig_point_ch0!=second_trig_point_ch1)||(second_trig_point_ch0!=second_trig_point_ch2)||(second_trig_point_ch2!=second_trig_point_ch1))
        {
            inst_name= "Ch0_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        if((epwm_sync_ch2!=epwm_sync_ch1)||(epwm_sync_ch2!=epwm_sync_ch0)||(epwm_sync_ch0!=epwm_sync_ch1))
        {
            inst_name= "Ch0_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        if((epwm_source_ch0!=epwm_source_ch1)||(epwm_source_ch2!=epwm_source_ch1)||((epwm_source_ch0!=epwm_source_ch2)))
        {
            inst_name= "Ch0_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel0, Channel1 and Channel2",
                inst, inst_name);
        }
        break;
    case 0:
    case 1:
    case 2:
    case 4:
    default:
        break;
  }
    
  let channel_id1 = 0;
  inst_name = "Enable_Channel_3";
  let ch3 = inst[inst_name];
  inst_name = "Enable_Channel_4";
  let ch4 = inst[inst_name];
  inst_name= "Enable_Channel_5";
  let ch5 = inst[inst_name];
  channel_id1 = (ch3|(ch4<<1)|(ch5<<2));

  inst_name= "Ch3_SDFM Clock";
  let clock_ch3 = inst[inst_name];
  inst_name= "Ch4_SDFM Clock";
  let clock_ch4 = inst[inst_name];
  inst_name= "Ch5_SDFM Clock";
  let clock_ch5 = inst[inst_name];

  inst_name= "Ch3_NC_OSR";
  let nosr_ch3 = inst[inst_name];
  inst_name= "Ch4_NC_OSR";
  let nosr_ch4 = inst[inst_name];
  inst_name= "Ch5_NC_OSR";
  let nosr_ch5 = inst[inst_name];

  inst_name= "Ch3_EnableDoubleUpdate";
  let double_update_ch3 = inst[inst_name];
  inst_name= "Ch4_EnableDoubleUpdate";
  let double_update_ch4 = inst[inst_name];
  inst_name= "Ch5_EnableDoubleUpdate";
  let double_update_ch5 = inst[inst_name];

  inst_name= "Ch3_EnableContinuousMode";
  let cont_mode_ch3 = inst[inst_name];
  inst_name= "Ch4_EnableContinuousMode";
  let cont_mode_ch4 = inst[inst_name];
  inst_name= "Ch5_EnableContinuousMode";
  let cont_mode_ch5 = inst[inst_name];

  inst_name= "Ch3_FirstTriggerPoint";
  let fist_trig_point_ch3 = inst[inst_name];
  inst_name= "Ch4_FirstTriggerPoint";
  let fist_trig_point_ch4 = inst[inst_name];
  inst_name= "Ch5_FirstTriggerPoint";
  let fist_trig_point_ch5 = inst[inst_name];

  inst_name= "Ch3_SecondTriggerPoint";
  let second_trig_point_ch3= inst[inst_name];
  inst_name= "Ch4_SecondTriggerPoint";
  let second_trig_point_ch4 = inst[inst_name];
  inst_name= "Ch5_SecondTriggerPoint";
  let second_trig_point_ch5 = inst[inst_name];
  
  inst_name= "Ch3_EPWM_SYNC";
  let epwm_sync_ch3= inst[inst_name];
  inst_name= "Ch4_EPWM_SYNC";
  let epwm_sync_ch4 = inst[inst_name];
  inst_name= "Ch5_EPWM_SYNC";
  let epwm_sync_ch5 = inst[inst_name];

  inst_name= "Ch3_Epwm_Source";
  let epwm_source_ch3 = inst[inst_name];
  inst_name= "Ch4_Epwm_Source";
  let epwm_source_ch4 = inst[inst_name];
  inst_name= "Ch5_Epwm_Source";
  let epwm_source_ch5 = inst[inst_name];
  
  switch (channel_id1)
  {
      
    case 3:
        if(clock_ch3 != clock_ch4 )
        {   
            inst_name= "Ch3_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel3 and Channel4",
                inst, inst_name);
        }
        if(nosr_ch3 != nosr_ch4 )
        {
            inst_name= "Ch3_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel3 and Channel4",
                inst, inst_name);
        }
        if(double_update_ch3!=double_update_ch4)
        {
            inst_name= "Ch3_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel3 and Channel4",
                inst, inst_name);
        }
        if(cont_mode_ch3!=cont_mode_ch4)
        {
            inst_name= "Ch3_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel3 and Channel4",
                inst, inst_name);
        }
        if(fist_trig_point_ch3!=fist_trig_point_ch4)
        {
            inst_name= "Ch3_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        if(second_trig_point_ch3!=second_trig_point_ch4)
        {
            inst_name= "Ch3_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        if(epwm_sync_ch3!=epwm_sync_ch4)
        {
            inst_name= "Ch3_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel3 and Channel4",
                inst, inst_name);
        }
        if(epwm_source_ch3!=epwm_source_ch4)
        {
            inst_name= "Ch3_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        break;
    case 5:
        if(clock_ch3 != clock_ch5 )
        {
            inst_name= "Ch3_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel3 and Channel5",
                inst, inst_name); 
        }  
        if(nosr_ch3 != nosr_ch5 )
        {
            inst_name= "Ch3_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel3 and Channel5",
                inst, inst_name); 
        }  
        if(double_update_ch3!=double_update_ch5)
        {
            inst_name= "Ch3_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel3 and Channel5",
                inst, inst_name);
        }
        if(cont_mode_ch3!=cont_mode_ch5)
        {
            inst_name= "Ch3_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel3 and Channel5",
                inst, inst_name);
        }
        if(fist_trig_point_ch3!=fist_trig_point_ch5)
        {
            inst_name= "Ch3_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        } 
        if(second_trig_point_ch3!=second_trig_point_ch5)
        {
            inst_name= "Ch3_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }    
        if(epwm_sync_ch3!=epwm_sync_ch5)
        {
            inst_name= "Ch3_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel3 and Channel5",
                inst, inst_name);
        } 
        if(epwm_source_ch3!=epwm_source_ch5)
        {
            inst_name= "Ch3_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }        
        break;
    case 6:
        if(clock_ch4 != clock_ch5 )
        {
            inst_name= "Ch4_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel4 and Channel5",
                inst, inst_name);
        }
        
        if(nosr_ch4 != nosr_ch5 )
        {
            inst_name= "Ch4_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel4 and Channel5",
                inst, inst_name);
        }

        if(double_update_ch4!=double_update_ch5)
        {
            inst_name= "Ch4_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel4 and Channel5 ",
                inst, inst_name);
        }
        if(cont_mode_ch4!=cont_mode_ch5)
        {
            inst_name= "Ch4_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel4 and Channel5",
                inst, inst_name);
        }
        if(fist_trig_point_ch4!=fist_trig_point_ch5)
        {
            inst_name= "Ch4_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        if(second_trig_point_ch5!=second_trig_point_ch4)
        {
            inst_name= "Ch4_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        if(epwm_sync_ch4!=epwm_sync_ch5)
        {
            inst_name= "Ch4_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel4 and Channel5",
                inst, inst_name);
        }
        if(epwm_source_ch5!=epwm_source_ch4)
        {
            inst_name= "Ch4_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        break;
    case 7:
        if((clock_ch3 != clock_ch4 )||(clock_ch3 != clock_ch5)||(clock_ch4 != clock_ch5))
        {
            inst_name= "Ch3_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel3 - 5",
                inst, inst_name);
        }
        if((nosr_ch3 != nosr_ch4)||(nosr_ch4 != nosr_ch5 )||(nosr_ch3 != nosr_ch5))
        {
            inst_name= "Ch3_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel3 - 5",
                inst, inst_name);
        }
        if((double_update_ch5 != double_update_ch4)||(double_update_ch5 != double_update_ch3)||(double_update_ch3 != double_update_ch4))
        {
            inst_name= "Ch3_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel3, Channel4 and Channel5 ",
                inst, inst_name);
        }
        if((cont_mode_ch5 != cont_mode_ch4)||(cont_mode_ch3 != cont_mode_ch4)||(cont_mode_ch5 != cont_mode_ch3))
        {
            inst_name= "Ch3_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel3, Channel4 and Channel5",
                inst, inst_name);  
        }
        if((fist_trig_point_ch3 != fist_trig_point_ch4)||(fist_trig_point_ch5 != fist_trig_point_ch4)||(fist_trig_point_ch3 != fist_trig_point_ch5))
        {
            inst_name= "Ch3_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        if((second_trig_point_ch3 != second_trig_point_ch4)||(second_trig_point_ch3 != second_trig_point_ch5)||(second_trig_point_ch4 != second_trig_point_ch5))
        {
            inst_name= "Ch3_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        if((epwm_sync_ch5 != epwm_sync_ch4)||(epwm_sync_ch5 != epwm_sync_ch3)||(epwm_sync_ch3 != epwm_sync_ch4))
        {
            inst_name= "Ch3_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        if((epwm_source_ch3 != epwm_source_ch4)||(epwm_source_ch5 != epwm_source_ch4)||((epwm_source_ch3 != epwm_source_ch5)))
        {
            inst_name= "Ch3_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel3, Channel4 and Channel5",
                inst, inst_name);
        }
        break;
    case 0:
    case 1:
    case 2:
    case 4:
    default:
          break;
  }

  let channel_id2 = 0;
  inst_name = "Enable_Channel_6";
  let ch6 = inst[inst_name];
  inst_name = "Enable_Channel_7";
  let ch7 = inst[inst_name];
  inst_name = "Enable_Channel_8";
  let ch8 = inst[inst_name];
  channel_id2 = (ch6|(ch7<<1)|(ch8<<2));
  
  inst_name= "Ch6_SDFM Clock";
  let clock_ch6 = inst[inst_name];
  inst_name= "Ch7_SDFM Clock";
  let clock_ch7 = inst[inst_name];
  inst_name= "Ch8_SDFM Clock";
  let clock_ch8 = inst[inst_name];
  
  inst_name= "Ch6_NC_OSR";
  let nosr_ch6 = inst[inst_name];
  inst_name= "Ch7_NC_OSR";
  let nosr_ch7 = inst[inst_name];
  inst_name= "Ch8_NC_OSR";
  let nosr_ch8 = inst[inst_name];

  inst_name= "Ch6_EnableDoubleUpdate";
  let double_update_ch6 = inst[inst_name];
  inst_name= "Ch7_EnableDoubleUpdate";
  let double_update_ch7 = inst[inst_name];
  inst_name= "Ch8_EnableDoubleUpdate";
  let double_update_ch8 = inst[inst_name];

  inst_name= "Ch6_EnableContinuousMode";
  let cont_mode_ch6 = inst[inst_name];
  inst_name= "Ch7_EnableContinuousMode";
  let cont_mode_ch7 = inst[inst_name];
  inst_name= "Ch8_EnableContinuousMode";
  let cont_mode_ch8 = inst[inst_name];

  inst_name= "Ch6_FirstTriggerPoint";
  let fist_trig_point_ch6 = inst[inst_name];
  inst_name= "Ch7_FirstTriggerPoint";
  let fist_trig_point_ch7 = inst[inst_name];
  inst_name= "Ch8_FirstTriggerPoint";
  let fist_trig_point_ch8 = inst[inst_name];

  inst_name= "Ch6_SecondTriggerPoint";
  let second_trig_point_ch6= inst[inst_name];
  inst_name= "Ch7_SecondTriggerPoint";
  let second_trig_point_ch7 = inst[inst_name];
  inst_name= "Ch8_SecondTriggerPoint";
  let second_trig_point_ch8 = inst[inst_name];
  
  inst_name= "Ch6_EPWM_SYNC";
  let epwm_sync_ch6= inst[inst_name];
  inst_name= "Ch7_EPWM_SYNC";
  let epwm_sync_ch7 = inst[inst_name];
  inst_name= "Ch8_EPWM_SYNC";
  let epwm_sync_ch8 = inst[inst_name];

  inst_name= "Ch6_Epwm_Source";
  let epwm_source_ch6 = inst[inst_name];
  inst_name= "Ch7_Epwm_Source";
  let epwm_source_ch7 = inst[inst_name];
  inst_name= "Ch8_Epwm_Source";
  let epwm_source_ch8 = inst[inst_name];

  switch (channel_id2)
  {
      
    case 3:
        if(clock_ch6 != clock_ch7 )
        {   
            inst_name= "Ch6_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel6 and Channel7",
                inst, inst_name);
        }
        if(nosr_ch6 != nosr_ch7 )
        {
            inst_name= "Ch6_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel6 and Channel7",
                inst, inst_name);
        }
        if(double_update_ch6!=double_update_ch7)
        {
            inst_name= "Ch6_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel6 and Channel7",
                inst, inst_name);
        }
        if(cont_mode_ch6!=cont_mode_ch7)
        {
            inst_name= "Ch6_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel6 and Channel7",
                inst, inst_name);
        }
        if(fist_trig_point_ch6!=fist_trig_point_ch7)
        {
            inst_name= "Ch6_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        if(second_trig_point_ch6!=second_trig_point_ch7)
        {
            inst_name= "Ch6_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        if(epwm_sync_ch6 != epwm_sync_ch7)
        {
            inst_name= "Ch6_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel6 and Channel7",
                inst, inst_name);
        }
        if(epwm_source_ch6 != epwm_source_ch7)
        {
            inst_name= "Ch6_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        break;
    case 5:
        if(clock_ch6 != clock_ch8 )
        {
            inst_name= "Ch6_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel6 and Channel8",
                inst, inst_name);  
        }
        if(nosr_ch6 != nosr_ch8 )
        {
            inst_name= "Ch6_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel6 and Channel8",
                inst, inst_name);  
        }
        if(double_update_ch6!=double_update_ch8)
        {
            inst_name= "Ch6_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel6 and Channel8",
                inst, inst_name);
        }
        if(cont_mode_ch6!=cont_mode_ch8)
        {
            inst_name= "Ch6_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel6 and Channel8",
                inst, inst_name);
        }
        if(fist_trig_point_ch6!=fist_trig_point_ch8)
        {
            inst_name= "Ch6_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        } 
        if(second_trig_point_ch6!=second_trig_point_ch8)
        {
            inst_name= "Ch6_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }    
        if(epwm_sync_ch6!=epwm_sync_ch8)
        {
            inst_name= "Ch6_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel6 and Channel8",
                inst, inst_name);
        } 
        if(epwm_source_ch0!=epwm_source_ch2)
        {
            inst_name= "Ch6_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        break;
    case 6:
        if(clock_ch7 != clock_ch8 )
        {
            inst_name= "Ch7_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel7 and Channel8",
                inst, inst_name);
        }
        
        if(nosr_ch7 != nosr_ch8 )
        {
            inst_name= "Ch7_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel7 and Channel8",
                inst, inst_name);
        }

        if(double_update_ch7 != double_update_ch8)
        {
            inst_name= "Ch7_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel7 and Channel8 ",
                inst, inst_name);
        }
        if(cont_mode_ch7!=cont_mode_ch8)
        {
            inst_name= "Ch7_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel7 and Channel8",
                inst, inst_name);
        }
        if(fist_trig_point_ch7 != fist_trig_point_ch8)
        {
            inst_name= "Ch7_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        if(second_trig_point_ch7!=second_trig_point_ch8)
        {
            inst_name= "Ch7_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        if(epwm_sync_ch7 != epwm_sync_ch8)
        {
            inst_name= "Ch7_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel7 and Channel8",
                inst, inst_name);
        }
        if(epwm_source_ch7 != epwm_source_ch8)
        {
            inst_name= "Ch7_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        break;
    case 7:
        if((clock_ch6 != clock_ch7 )||(clock_ch6 != clock_ch8)||(clock_ch7!=clock_ch8))
        {
            inst_name= "Ch6_SDFM Clock";
            validation.logError(
                "SDFM clock value should be same for Channel6-8",
                inst, inst_name);
        }
        if((nosr_ch6 != nosr_ch7)||(nosr_ch7 != nosr_ch8 )||(nosr_ch6 != nosr_ch8))
        {
            inst_name= "Ch6_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channel6 - 8",
                inst, inst_name);
        }
        if((double_update_ch8!=double_update_ch7)||(double_update_ch8!=double_update_ch6)||(double_update_ch6!=double_update_ch7))
        {
            inst_name= "Ch6_EnableDoubleUpdate";
            validation.logError(
                "Enable double update for Channel6, Channel7 and Channel8 ",
                inst, inst_name);
        }
        if((cont_mode_ch8!=cont_mode_ch7)||(cont_mode_ch6!=cont_mode_ch7)||(cont_mode_ch8!=cont_mode_ch6))
        {
            inst_name= "Ch6_EnableContinuousMode";
            validation.logError(
                "Enable Continuous mode for Channel6, Channel7 and Channel8",
                inst, inst_name);  
        }
        if((fist_trig_point_ch6!=fist_trig_point_ch7)||(fist_trig_point_ch8!=fist_trig_point_ch7)||(fist_trig_point_ch6!=fist_trig_point_ch8))
        {
            inst_name= "Ch6_FirstTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        if((second_trig_point_ch6!=second_trig_point_ch7)||(second_trig_point_ch6!=second_trig_point_ch8)||(second_trig_point_ch8!=second_trig_point_ch7))
        {
            inst_name= "Ch6_SecondTriggerPoint";
            validation.logError(
                "First Trigger point should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        if((epwm_sync_ch8!=epwm_sync_ch7)||(epwm_sync_ch8!=epwm_sync_ch6)||(epwm_sync_ch6!=epwm_sync_ch7))
        {
            inst_name= "Ch6_EPWM_SYNC";
            validation.logError(
                "Enable EPWM sync for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        if((epwm_source_ch6!=epwm_source_ch7)||(epwm_source_ch8!=epwm_source_ch7)||((epwm_source_ch6!=epwm_source_ch8)))
        {
            inst_name= "Ch6_Epwm_Source";
            validation.logError(
                "EPWM sync source should be same for Channel6, Channel7 and Channel8",
                inst, inst_name);
        }
        break;
    case 0:
    case 1:
    case 2:
    case 4:
    default:
          break;
  }  
}


exports =
{
    onValidate : onValidate,
}