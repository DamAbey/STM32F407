#include "pdm_fir.h"

/* PDM FIR filter.
 * The source frequency is expected to be 1024kHz so we are receiving 16 bit words at 64kHz rate MSB first.
 * The filter cutoff frequency is 8kHz.
 */

/* The following file contains tables generated by pdm_fir.py script. You can easily customize filter
 * parameters by modifying the pdm_fir.py script and regenerating tables.
 */
#include "pdm_fir_.h"

/* Initialize filter */
void pdm_fir_flt_init(struct pdm_fir_filter* f)
{
	int t;
	f->next_tap = 0;
	for (t = 0; t < PDM_FTL_TAPS; ++t)
		f->buffer[t] = 0x5555;
}

/* Put 16 bits MSB first */
void pdm_fir_flt_put(struct pdm_fir_filter* f, uint16_t bits)
{
	f->buffer[f->next_tap] = bits;
	if (++f->next_tap >= PDM_FTL_TAPS)
		f->next_tap = 0;
}

/* Retrieve output value. May be called at any rate since it does not change the filter state.
 * The output ranges from -(2**(out_bits-1)) to +(2**(out_bits-1)). Those values correspond to
 * all 0 or all 1 input signal. Note that the output value may still exceed this range so caller
 * should truncate return value on its own if necessary.
 */


int pdm_fir_flt_get(struct pdm_fir_filter const* f, int out_bits)
{
	int t, i = 0, tot = 0, j=0, bitval=0;

	for (t = (f->next_tap);;) {
		uint16_t v = f->buffer[t];
			for(j=0;j<16;j++)
			{
				//bitval=v&(1<<15);
				bitval=((v&(1<<(15-j)))>>(15-j));
				if(bitval==1){
				tot=tot+tap_coeff[i*15+j];
				}
				else{
				tot=tot-tap_coeff[i*15+j];
				}

			}

		if (++t >= PDM_FTL_TAPS)
			t = 0;
		if (t == f->next_tap)
		{
			break;
		}
		i=i+1;
	}

	return tot >> (PDM_FTL_SCALE_BITS - out_bits + 1);
}



/*

int pdm_fir_flt_get(struct pdm_fir_filter const* f, int out_bits)
{
	int t, i = 0, tot = 0;
	for (t = f->next_tap;;) {
		uint16_t v = f->buffer[t];
		tot += byte_coeff[i++][(uint8_t)(v>>8)]; // (uint8_t)(v>>8) 8 MSBs shifted
		tot += byte_coeff[i++][(uint8_t)(v)];// (uint8_t)(v) 8 LSBs
		if (++t >= PDM_FTL_TAPS)
			t = 0;
		if (t == f->next_tap)
			break;
	}

	return tot >> (PDM_FTL_SCALE_BITS - out_bits + 1);
}*/


void lowPassFrequency(int16_t* input, int16_t* output, uint16_t points)
{
    double RC = 1.0/(1000*2*3.14); //1kHz cutoff
    double dt = 1.0/32000; //32kHz sample rate
    double alpha = dt/(RC+dt);
    output[0] = input[0];
    for(int i = 1; i < points; ++i)
    {
        output[i] = output[i-1] + (alpha*(input[i] - output[i-1]));
    }
}
