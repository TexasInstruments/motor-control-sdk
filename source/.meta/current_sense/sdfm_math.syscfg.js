let common   = system.getScript("/common");

function getFilterOutputRange(FilterType, OSR)
{
  let Sinc1 = [];
  let Sinc2 = [];
  let Sinc3 = [];

  Sinc1[OSR] 	= OSR;
  Sinc2[OSR] 	= OSR*OSR;
  Sinc3[OSR]    = OSR*OSR*OSR;

  let filter_32bit_Output = 0;

  switch(FilterType)
  {
  case '2':
    filter_32bit_Output = Sinc1[OSR];
    break;

  case '1':
    filter_32bit_Output = Sinc2[OSR];
    break;
  case '0':
    filter_32bit_Output = Sinc3[OSR];
    break;

  }

  return filter_32bit_Output;
  
}

exports =
{
	getFilterOutputRange : getFilterOutputRange,
}
