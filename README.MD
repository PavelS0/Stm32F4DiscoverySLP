


#ЗАМЕТКИ:
https://electronix.ru/forum/index.php?app=forums&module=forums&controller=topic&id=154447#comments
При децимейшн факторе 64 нужно скормить 4 полуслова.
DECIMATION_FACTOR_64 - это 64 бита, одно полуслово это 16 бит. Соответственно 4 полуслова.
в настройках PDM фильтра поставить outputSamplesNumber = 1;

Надо завести DMA :
HAL_I2S_Receive_DMA(&hi2s2, PDM_BUFFER, 8);
В обработчике по заполнению половины буфера и полного буфера скармливать
PDM фильтру соответственно первые 4 полуслова, и последние 4 полуслова.



http://www.chibios.com/forum/viewtopic.php?t=2199
I also posted this in ST's E2E support but there doesn't seem to be anyone about who's worked with this so thought I would try my luck here.

I've been racking my head trying to make sense of how to correctly configure the I2S Hardware to interface with the MP45DT02 microphone. I've been looking at ST's waverecorder.c for inspiration but I think some iffy comments are really throwing me off. I was hoping someone here would be able to help me make sense of the configuration - I basically want to be able to understand how to select the appropriate values for the registers.

So, say I desire to ultimately get 16 bit 16 kHz PCM audio. This means, with 64 decimation, I need to sample the PDM microphone at 64 * 16 kHz = 1.024 MHz.

The I2S needs to be configured to do this and I understand how (in ST's example):
HSE_Frequency = 8 MHz
PLL_M = 8
PLLI2S_N = 258
PLLI2S_R = 3
Can give at 86 MHz clock to the I2S hardware. Its here where it all goes to pot. How do I configure I2S with an input of 86 MHz (or otherwise) to sample PDM at 1.024 MHz.

Would I be right in saying a 1.024 MHz clock for PDM is the same as considering 16 bit I2S to be sampled at 64 kHz? ST's code comments throw me at this point as they state:
* - Master clock output (MCKO): ON
* - Audio sampling freq (KHz) : 48

but then the I2S init function gets:
I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
I2S_InitStructure.I2S_AudioFreq = 32000;