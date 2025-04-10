/*
	In this GPS SDR SIM file, two functionalities are added:
	1. Gaussian Noise Control:
	 - Test with gain_dB 10,20,30,....,100
	 - No SV detected at 100dB
	 - SV detected at 10dB
	2. GPS Outage Control:
	 - Test 1:
	   Seconds: 	0		75		105		120
	   Observed:	0	48	77		107		120
	                 <--><--><------><------->   
                       .   .     .       .
                 No lock   .     .       .
				         PVT     .       .
				            Outage       .
				                        SV
	 - Test 2:
	   Seconds:		0		75		105				180
	   Observed:	0	48	77		108		138		180
	                <---><--><------><------><------->
	                  .   .      .      .        .
	            No lock   .      .      .        .
	                    PVT      .      .        .
	                        Outage      .        .
	                              No lock        .
	                                           sPVT
	3. GPS Outage, Gaussian Noise and Simulator Options all are integrated in this source file
	4. The GPS Outage loops have been modified to encorporate the Simulator Gain Options
	5. The Gaussian Noise can be called via terminal, Example: -G 110.5
	6. The Outage Start and Stop Times can be called via terminal, Example: -O 10,20
	7. -G Gain vs Hackrf dBfs readout experiment:
	8. 	at 50dB  -> -12.4 dBfs, SV detected, PVT acquired at 13 sec from cold start
	    at 60dB  -> -12.4 dBfs, SV detected, PVT acquired at 125 sec from cold start
		at 70dB  -> -12.1 dBfs, SV detected, No PVT acquired from cold start (100s RF)
		at 80dB  -> -10.3 dBfs, SV detected, No PVT acquired from cold start (100s RF)
		at 90dB  -> -4.20 dBfs, SV detected, No PVT acquired from cold start (100s RF)
		at 100dB -> -1.7 dBfs, no SV detected till 100s
		at 110dB -> -1.7 dBfs, no SV detected till 100s
	9. -G Gain vs Spectrum Analyzer Response:
	
*/

#define _CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#ifdef _WIN32
#include "getopt.h"
#else
#include <unistd.h>
#endif
#include "gpssim.h"

//ADDED BY ME:
int pathLossMode;
int indexPLM2 = -1;
//int chanID[16];
char pathLossModeChar[];
char chanID[50];
char chanStr[200];
char chanMode[50];
char fixStrChar[10];
char ionoChangeChar[10];
char outage_times[2];

int chanIDAr[16];
float chanStrAr[16];
int chanModeAr[16];
int outage_times_Ar[2];
double fixStr;
double ionoChange;
float gain_factor;

int sinTable512[] = {
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250,
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2
};

int cosTable512[] = {
	 250, 250, 250, 250, 250, 249, 249, 249, 249, 248, 248, 248, 247, 247, 246, 245,
	 245, 244, 244, 243, 242, 241, 241, 240, 239, 238, 237, 236, 235, 234, 233, 232,
	 230, 229, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209,
	 207, 205, 204, 202, 200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178,
	 176, 173, 171, 169, 167, 164, 162, 160, 157, 155, 153, 150, 148, 145, 143, 140,
	 138, 135, 132, 130, 127, 125, 122, 119, 116, 114, 111, 108, 105, 103, 100,  97,
	  94,  91,  89,  86,  83,  80,  77,  74,  71,  68,  65,  62,  59,  56,  53,  50,
	  47,  44,  41,  38,  35,  32,  29,  26,  23,  20,  17,  14,  11,   8,   5,   2,
	  -2,  -5,  -8, -11, -14, -17, -20, -23, -26, -29, -32, -35, -38, -41, -44, -47,
	 -50, -53, -56, -59, -62, -65, -68, -71, -74, -77, -80, -83, -86, -89, -91, -94,
	 -97,-100,-103,-105,-108,-111,-114,-116,-119,-122,-125,-127,-130,-132,-135,-138,
	-140,-143,-145,-148,-150,-153,-155,-157,-160,-162,-164,-167,-169,-171,-173,-176,
	-178,-180,-182,-184,-186,-188,-190,-192,-194,-196,-198,-200,-202,-204,-205,-207,
	-209,-210,-212,-214,-215,-217,-218,-220,-221,-223,-224,-225,-227,-228,-229,-230,
	-232,-233,-234,-235,-236,-237,-238,-239,-240,-241,-241,-242,-243,-244,-244,-245,
	-245,-246,-247,-247,-248,-248,-248,-249,-249,-249,-249,-250,-250,-250,-250,-250,
	-250,-250,-250,-250,-250,-249,-249,-249,-249,-248,-248,-248,-247,-247,-246,-245,
	-245,-244,-244,-243,-242,-241,-241,-240,-239,-238,-237,-236,-235,-234,-233,-232,
	-230,-229,-228,-227,-225,-224,-223,-221,-220,-218,-217,-215,-214,-212,-210,-209,
	-207,-205,-204,-202,-200,-198,-196,-194,-192,-190,-188,-186,-184,-182,-180,-178,
	-176,-173,-171,-169,-167,-164,-162,-160,-157,-155,-153,-150,-148,-145,-143,-140,
	-138,-135,-132,-130,-127,-125,-122,-119,-116,-114,-111,-108,-105,-103,-100, -97,
	 -94, -91, -89, -86, -83, -80, -77, -74, -71, -68, -65, -62, -59, -56, -53, -50,
	 -47, -44, -41, -38, -35, -32, -29, -26, -23, -20, -17, -14, -11,  -8,  -5,  -2,
	   2,   5,   8,  11,  14,  17,  20,  23,  26,  29,  32,  35,  38,  41,  44,  47,
	  50,  53,  56,  59,  62,  65,  68,  71,  74,  77,  80,  83,  86,  89,  91,  94,
	  97, 100, 103, 105, 108, 111, 114, 116, 119, 122, 125, 127, 130, 132, 135, 138,
	 140, 143, 145, 148, 150, 153, 155, 157, 160, 162, 164, 167, 169, 171, 173, 176,
	 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 200, 202, 204, 205, 207,
	 209, 210, 212, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 229, 230,
	 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242, 243, 244, 244, 245,
	 245, 246, 247, 247, 248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 250
};

double logTable512[512] = {
    -6.907755, -5.824260, -5.316485, -4.981324, -4.730738, -4.530531, -4.363798, -4.220933,
    -4.095951, -3.984867, -3.884899, -3.794023, -3.710721, -3.633828, -3.562428, -3.495788,
    -3.433312, -3.374512, -3.318977, -3.266365, -3.216384, -3.168782, -3.123343, -3.079880,
    -3.038227, -2.998240, -2.959791, -2.922766, -2.887062, -2.852590, -2.819266, -2.787017,
    -2.755776, -2.725481, -2.696077, -2.667513, -2.639743, -2.612722, -2.586413, -2.560778,
    -2.535784, -2.511399, -2.487595, -2.464345, -2.441622, -2.419405, -2.397670, -2.376398,
    -2.355569, -2.335165, -2.315169, -2.295565, -2.276338, -2.257473, -2.238958, -2.220780,
    -2.202926, -2.185385, -2.168147, -2.151200, -2.134536, -2.118146, -2.102019, -2.086149,
    -2.070526, -2.055144, -2.039995, -2.025072, -2.010368, -1.995878, -1.981594, -1.967512,
    -1.953625, -1.939928, -1.926416, -1.913085, -1.899929, -1.886944, -1.874125, -1.861468,
    -1.848970, -1.836626, -1.824432, -1.812386, -1.800482, -1.788719, -1.777093, -1.765600,
    -1.754237, -1.743003, -1.731893, -1.720905, -1.710037, -1.699286, -1.688648, -1.678123,
    -1.667708, -1.657400, -1.647197, -1.637097, -1.627098, -1.617198, -1.607395, -1.597687,
    -1.588073, -1.578550, -1.569117, -1.559772, -1.550514, -1.541340, -1.532250, -1.523242,
    -1.514315, -1.505466, -1.496695, -1.487999, -1.479380, -1.470834, -1.462360, -1.453957,
    -1.445625, -1.437361, -1.429165, -1.421036, -1.412972, -1.404973, -1.397037, -1.389163,
    -1.381352, -1.373600, -1.365909, -1.358276, -1.350701, -1.343182, -1.335720, -1.328314,
    -1.320961, -1.313662, -1.306417, -1.299223, -1.292081, -1.284989, -1.277947, -1.270955,
    -1.264011, -1.257115, -1.250266, -1.243464, -1.236708, -1.229997, -1.223331, -1.216708,
    -1.210130, -1.203595, -1.197101, -1.190650, -1.184241, -1.177872, -1.171543, -1.165254,
    -1.159004, -1.152794, -1.146621, -1.140487, -1.134390, -1.128329, -1.122306, -1.116318,
    -1.110366, -1.104449, -1.098567, -1.092720, -1.086906, -1.081126, -1.075379, -1.069665,
    -1.063984, -1.058335, -1.052717, -1.047131, -1.041576, -1.036051, -1.030557, -1.025093,
    -1.019658, -1.014253, -1.008877, -1.003530, -0.998211, -0.992920, -0.987657, -0.982422,
    -0.977214, -0.972033, -0.966879, -0.961751, -0.956650, -0.951573, -0.946523, -0.941498,
    -0.936499, -0.931524, -0.926574, -0.921648, -0.916747, -0.911869, -0.907015, -0.902185,
    -0.897379, -0.892596, -0.887836, -0.883099, -0.878385, -0.873693, -0.869025, -0.864378,
    -0.859754, -0.855152, -0.850572, -0.846014, -0.841478, -0.836963, -0.832470, -0.827998,
    -0.823547, -0.819117, -0.814707, -0.810319, -0.805950, -0.801602, -0.797274, -0.792967,
    -0.788679, -0.784411, -0.780163, -0.775935, -0.771726, -0.767536, -0.763366, -0.759215,
    -0.755083, -0.750970, -0.746876, -0.742801, -0.738745, -0.734707, -0.730688, -0.726688,
    -0.722706, -0.718742, -0.714796, -0.710869, -0.706960, -0.703069, -0.699195, -0.695340,
    -0.691502, -0.687683, -0.683880, -0.680096, -0.676329, -0.672579, -0.668847, -0.665132,
    -0.661434, -0.657753, -0.654089, -0.650443, -0.646813, -0.643200, -0.639604, -0.636025,
    -0.632462, -0.628916, -0.625387, -0.621874, -0.618378, -0.614898, -0.611434, -0.607986,
    -0.604555, -0.601139, -0.597740, -0.594356, -0.590988, -0.587636, -0.584299, -0.580978,
    -0.577673, -0.574382, -0.571108, -0.567849, -0.564604, -0.561375, -0.558161, -0.554962,
    -0.551778, -0.548608, -0.545454, -0.542313, -0.539188, -0.536076, -0.532979, -0.529896,
    -0.526828, -0.523774, -0.520734, -0.517708, -0.514696, -0.511698, -0.508713, -0.505743,
    -0.502786, -0.499843, -0.496913, -0.493997, -0.491094, -0.488205, -0.485329, -0.482466,
    -0.479617, -0.476780, -0.473956, -0.471146, -0.468348, -0.465563, -0.462791, -0.460031,
    -0.457285, -0.454550, -0.451829, -0.449119, -0.446422, -0.443737, -0.441064, -0.438403,
    -0.435755, -0.433118, -0.430493, -0.427880, -0.425278, -0.422689, -0.420111, -0.417544,
    -0.414989, -0.412445, -0.409913, -0.407392, -0.404882, -0.402384, -0.399896, -0.397420,
    -0.394954, -0.392500, -0.390057, -0.387624, -0.385203, -0.382792, -0.380392, -0.378002,
    -0.375623, -0.373255, -0.370897, -0.368550, -0.366213, -0.363886, -0.361570, -0.359264,
    -0.356968, -0.354682, -0.352407, -0.350141, -0.347886, -0.345640, -0.343405, -0.341180,
    -0.338964, -0.336758, -0.334562, -0.332375, -0.330198, -0.328031, -0.325873, -0.323725,
    -0.321587, -0.319458, -0.317338, -0.315228, -0.313127, -0.311035, -0.308952, -0.306879,
    -0.304815, -0.302760, -0.300714, -0.298677, -0.296649, -0.294630, -0.292620, -0.290619,
    -0.288626, -0.286643, -0.284668, -0.282702, -0.280745, -0.278796, -0.276856, -0.274925,
    -0.273002, -0.271088, -0.269182, -0.267285, -0.265396, -0.263516, -0.261644, -0.259780,
    -0.257924, -0.256077, -0.254238, -0.252407, -0.250584, -0.248769, -0.246962, -0.245164,
    -0.243373, -0.241590, -0.239815, -0.238048, -0.236288, -0.234537, -0.232793, -0.231057,
    -0.229329, -0.227608, -0.225895, -0.224189, -0.222492, -0.220801, -0.219118, -0.217443,
    -0.215775, -0.214114, -0.212461, -0.210815, -0.209176, -0.207545, -0.205921, -0.204303,
    -0.202694, -0.201091, -0.199495, -0.197906, -0.196324, -0.194749, -0.193181, -0.191619,
    -0.190065, -0.188518, -0.186977, -0.185443, -0.183916, -0.182396, -0.180882, -0.179375,
    -0.177874, -0.176380, -0.174892, -0.173411, -0.171936, -0.170468, -0.169006, -0.167550,
    -0.166101, -0.164658, -0.163221, -0.161791, -0.160367, -0.158948, -0.157537, -0.156130,
    -0.154730, -0.153336, -0.151948, -0.150566, -0.149189, -0.147818, -0.146453, -0.145094,
    -0.143740, -0.142392, -0.141049, -0.139712, -0.138380, -0.137054, -0.135733, -0.134418,
    -0.133107, -0.131802, -0.130502, -0.129207, -0.127917, -0.126633, -0.125353, -0.124079,
    -0.122809, -0.121544, -0.120284, -0.119029, -0.117778, -0.116533, -0.115292, -0.114055,
    -0.112823, -0.111596, -0.110373, -0.109155, -0.107942, -0.106733, -0.105528, -0.104327,
    -0.103131, -0.101939, -0.100751, -0.099568, -0.098388, -0.097213, -0.096042, -0.094875,
    -0.093712, -0.092553, -0.091398, -0.090246, -0.089098, -0.087955, -0.086815, -0.085679,
    -0.084547, -0.083418, -0.082293, -0.081171, -0.080053, -0.078938, -0.077827, -0.076720,
    -0.075616, -0.074515, -0.073418, -0.072324, -0.071233, -0.070146, -0.069062, -0.067981,
    -0.066904, -0.065830, -0.064759, -0.063691, -0.062627, -0.061565, -0.060507, -0.059452,
    -0.058400, -0.057351, -0.056305, -0.055262, -0.054221, -0.053184, -0.052150, -0.051118,
    -0.050090, -0.049064, -0.048041, -0.047021, -0.046003, -0.044988, -0.043975, -0.042965,
    -0.041958, -0.040953, -0.039951, -0.038951, -0.037954, -0.036960, -0.035967, -0.034978,
    -0.033990, -0.033005, -0.032022, -0.031041, -0.030063, -0.029087, -0.028113, -0.027142,
    -0.026172, -0.025205, -0.024240, -0.023276, -0.022315, -0.021357, -0.020400, -0.019445,
    -0.018492, -0.017541, -0.016592, -0.015645, -0.014700, -0.013757, -0.012816, -0.011877,
    -0.010940, -0.010004, -0.009071, -0.008139, -0.007209, -0.006281, -0.005355, -0.004430,
    -0.003508, -0.002586, -0.001667, -0.000749, 0.000000
};

//double gain_dB_Ar[20] = {
//	0.000000, 3.162277, 10.000000, 31.622776, 100.000000, 316.227766, 1000.000000,
//	3162.277660, 10000.000000, 31,622.776601, 100000.000000, 316,227.766016,
//	1000000.000000, 10000000.000000, 100000000.000000, 1000000000.000000,
//	3162277660.168379, 10000000000.000000
//};

// Receiver antenna attenuation in dB for boresight angle = 0:5:180 [deg]
double ant_pat_db[37] = {
	 0.00,  0.00,  0.22,  0.44,  0.67,  1.11,  1.56,  2.00,  2.44,  2.89,  3.56,  4.22,
	 4.89,  5.56,  6.22,  6.89,  7.56,  8.22,  8.89,  9.78, 10.67, 11.56, 12.44, 13.33,
	14.44, 15.56, 16.67, 17.78, 18.89, 20.00, 21.33, 22.67, 24.00, 25.56, 27.33, 29.33,
	31.56
};

#include <math.h>  // Required for sqrt and pow functions

// Gaussian Noise Generator using lookup tables for sin, cos, and log
double generate_gaussian_noise(double mean, double stddev, double gain_dB) {
    // Generate two uniform random values
    double u1 = rand() / (double)RAND_MAX; // Uniform(0,1] random doubles
    double u2 = rand() / (double)RAND_MAX;

    // Scale u1 to index into logTable512
    int log_index = (int)(u1 * 511) % 512;
    double log_value = logTable512[log_index]; // Use lookup table for log

    // Calculate f1 using the lookup log value
    double f1 = sqrt(-2.0 * log_value);

    // Scale u2 to index into sinTable512 and cosTable512
    int index = (int)(u2 * 511) % 512;
    double sin_value = sinTable512[index] / 250.0; // Normalize to original sine range [-1, 1]
    double cos_value = cosTable512[index] / 250.0; // Normalize to original cosine range [-1, 1]

    // Calculate g1 using lookup values for cos
    double g1 = f1 * cos_value; // Gaussian~N(0,1)

    // Convert dB gain to linear gain
    double gain_linear = pow(10, gain_dB / 20.0);
    //double gain_linear = gain_dB/10;

    // Return the Gaussian noise scaled by gain and standard deviation
    return gain_linear * (mean + stddev * g1);
    //return mean + gain_linear * g1;
}


int allocatedSat[MAX_SAT];

double xyz[USER_MOTION_SIZE][3];

/*! \brief Subtract two vectors of double
 *  \param[out] y Result of subtraction
 *  \param[in] x1 Minuend of subtraction
 *  \param[in] x2 Subtrahend of subtraction
 */
void subVect(double *y, const double *x1, const double *x2)
{
	y[0] = x1[0]-x2[0];
	y[1] = x1[1]-x2[1];
	y[2] = x1[2]-x2[2];

	return;
}

/*! \brief Compute Norm of Vector
 *  \param[in] x Input vector
 *  \returns Length (Norm) of the input vector
 */
double normVect(const double *x)
{
	return(sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]));
}

/*! \brief Compute dot-product of two vectors
 *  \param[in] x1 First multiplicand
 *  \param[in] x2 Second multiplicand
 *  \returns Dot-product of both multiplicands
 */
double dotProd(const double *x1, const double *x2)
{
	return(x1[0]*x2[0]+x1[1]*x2[1]+x1[2]*x2[2]);
}

/* !\brief generate the C/A code sequence for a given Satellite Vehicle PRN
 *  \param[in] prn PRN number of the Satellite Vehicle
 *  \param[out] ca Caller-allocated integer array of 1023 bytes
 */
void codegen(int *ca, int prn) //prn is the satellite ID 1 till 32
{
	int delay[] = {
		  5,   6,   7,   8,  17,  18, 139, 140, 141, 251,
		252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
		473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
		861, 862};

	int g1[CA_SEQ_LEN], g2[CA_SEQ_LEN];
	int r1[N_DWRD_SBF], r2[N_DWRD_SBF];
	int c1, c2;
	int i,j;

	if (prn<1 || prn>32)
		return;

	for (i=0; i<N_DWRD_SBF; i++)
		r1[i] = r2[i] = -1;

	for (i=0; i<CA_SEQ_LEN; i++)
	{
		g1[i] = r1[9];
		g2[i] = r2[9];
		c1 = r1[2]*r1[9];
		c2 = r2[1]*r2[2]*r2[5]*r2[7]*r2[8]*r2[9];

		for (j=9; j>0; j--)
		{
			r1[j] = r1[j-1];
			r2[j] = r2[j-1];
		}
		r1[0] = c1;
		r2[0] = c2;
	}

	for (i=0,j=CA_SEQ_LEN-delay[prn-1]; i<CA_SEQ_LEN; i++,j++)
		ca[i] = (1-g1[i]*g2[j%CA_SEQ_LEN])/2;

	return;
}

/*! \brief Convert a UTC date into a GPS date
 *  \param[in] t input date in UTC form
 *  \param[out] g output date in GPS form
 */
void date2gps(const datetime_t *t, gpstime_t *g)
{
	int doy[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
	int ye;
	int de;
	int lpdays;

	ye = t->y - 1980;

	// Compute the number of leap days since Jan 5/Jan 6, 1980.
	lpdays = ye/4 + 1;
	if ((ye%4)==0 && t->m<=2)
		lpdays--;

	// Compute the number of days elapsed since Jan 5/Jan 6, 1980.
	de = ye*365 + doy[t->m-1] + t->d + lpdays - 6;

	// Convert time to GPS weeks and seconds.
	g->week = de / 7;
	g->sec = (double)(de%7)*SECONDS_IN_DAY + t->hh*SECONDS_IN_HOUR
		+ t->mm*SECONDS_IN_MINUTE + t->sec;

	return;
}

void gps2date(const gpstime_t *g, datetime_t *t)
{
	// Convert Julian day number to calendar date
	int c = (int)(7*g->week + floor(g->sec/86400.0)+2444245.0) + 1537;
	int d = (int)((c-122.1)/365.25);
	int e = 365*d + d/4;
	int f = (int)((c-e)/30.6001);

	t->d = c - e - (int)(30.6001*f);
	t->m = f - 1 - 12*(f/14);
	t->y = d - 4715 - ((7 + t->m)/10);

	t->hh = ((int)(g->sec/3600.0))%24;
	t->mm = ((int)(g->sec/60.0))%60;
	t->sec = g->sec - 60.0*floor(g->sec/60.0);

	return;
}

/*! \brief Convert Earth-centered Earth-fixed (ECEF) into Lat/Long/Height
 *  \param[in] xyz Input Array of X, Y and Z ECEF coordinates
 *  \param[out] llh Output Array of Latitude, Longitude and Height
 */
void xyz2llh(const double *xyz, double *llh)
{
	double a,eps,e,e2;
	double x,y,z;
	double rho2,dz,zdz,nh,slat,n,dz_new;

	a = WGS84_RADIUS;
	e = WGS84_ECCENTRICITY;

	eps = 1.0e-3;
	e2 = e*e;

	if (normVect(xyz)<eps)
	{
		// Invalid ECEF vector
		llh[0] = 0.0;
		llh[1] = 0.0;
		llh[2] = -a;

		return;
	}

	x = xyz[0];
	y = xyz[1];
	z = xyz[2];

	rho2 = x*x + y*y;
	dz = e2*z;

	while (1)
	{
		zdz = z + dz;
		nh = sqrt(rho2 + zdz*zdz);
		slat = zdz / nh;
		n = a / sqrt(1.0-e2*slat*slat);
		dz_new = n*e2*slat;

		if (fabs(dz-dz_new) < eps)
			break;

		dz = dz_new;
	}

	llh[0] = atan2(zdz, sqrt(rho2));
	llh[1] = atan2(y, x);
	llh[2] = nh - n;

	return;
}

/*! \brief Convert Lat/Long/Height into Earth-centered Earth-fixed (ECEF)
 *  \param[in] llh Input Array of Latitude, Longitude and Height
 *  \param[out] xyz Output Array of X, Y and Z ECEF coordinates
 */
void llh2xyz(const double *llh, double *xyz)
{
	double n;
	double a;
	double e;
	double e2;
	double clat;
	double slat;
	double clon;
	double slon;
	double d,nph;
	double tmp;

	a = WGS84_RADIUS;
	e = WGS84_ECCENTRICITY;
	e2 = e*e;

	clat = cos(llh[0]);
	slat = sin(llh[0]);
	clon = cos(llh[1]);
	slon = sin(llh[1]);
	d = e*slat;

	n = a/sqrt(1.0-d*d);
	nph = n + llh[2];

	tmp = nph*clat;
	xyz[0] = tmp*clon;
	xyz[1] = tmp*slon;
	xyz[2] = ((1.0-e2)*n + llh[2])*slat;

	return;
}

/*! \brief Compute the intermediate matrix for LLH to ECEF
 *  \param[in] llh Input position in Latitude-Longitude-Height format
 *  \param[out] t Three-by-Three output matrix
 */
void ltcmat(const double *llh, double t[3][3])
{
	double slat, clat;
	double slon, clon;

	slat = sin(llh[0]);
	clat = cos(llh[0]);
	slon = sin(llh[1]);
	clon = cos(llh[1]);

	t[0][0] = -slat*clon;
	t[0][1] = -slat*slon;
	t[0][2] = clat;
	t[1][0] = -slon;
	t[1][1] = clon;
	t[1][2] = 0.0;
	t[2][0] = clat*clon;
	t[2][1] = clat*slon;
	t[2][2] = slat;

	return;
}

/*! \brief Convert Earth-centered Earth-Fixed to ?
 *  \param[in] xyz Input position as vector in ECEF format
 *  \param[in] t Intermediate matrix computed by \ref ltcmat
 *  \param[out] neu Output position as North-East-Up format
 */
void ecef2neu(const double *xyz, double t[3][3], double *neu)
{
	neu[0] = t[0][0]*xyz[0] + t[0][1]*xyz[1] + t[0][2]*xyz[2];
	neu[1] = t[1][0]*xyz[0] + t[1][1]*xyz[1] + t[1][2]*xyz[2];
	neu[2] = t[2][0]*xyz[0] + t[2][1]*xyz[1] + t[2][2]*xyz[2];

	return;
}

/*! \brief Convert North-East-Up to Azimuth + Elevation
 *  \param[in] neu Input position in North-East-Up format
 *  \param[out] azel Output array of azimuth + elevation as double
 */
void neu2azel(double *azel, const double *neu)
{
	double ne;

	azel[0] = atan2(neu[1],neu[0]);
	if (azel[0]<0.0)
		azel[0] += (2.0*PI);

	ne = sqrt(neu[0]*neu[0] + neu[1]*neu[1]);
	azel[1] = atan2(neu[2], ne);

	return;
}

/*! \brief Compute Satellite position, velocity and clock at given time
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at which position is to be computed
 *  \param[out] pos Computed position (vector)
 *  \param[out] vel Computed velocity (vector)
 *  \param[clk] clk Computed clock
 */
void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk)
{
	// Computing Satellite Velocity using the Broadcast Ephemeris
	// http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

	double tk;
	double mk;
	double ek;
	double ekold;
	double ekdot;
	double cek,sek;
	double pk;
	double pkdot;
	double c2pk,s2pk;
	double uk;
	double ukdot;
	double cuk,suk;
	double ok;
	double sok,cok;
	double ik;
	double ikdot;
	double sik,cik;
	double rk;
	double rkdot;
	double xpk,ypk;
	double xpkdot,ypkdot;

	double relativistic, OneMinusecosE, tmp;

	tk = g.sec - eph.toe.sec;

	if(tk>SECONDS_IN_HALF_WEEK)
		tk -= SECONDS_IN_WEEK;
	else if(tk<-SECONDS_IN_HALF_WEEK)
		tk += SECONDS_IN_WEEK;

	mk = eph.m0 + eph.n*tk;
	ek = mk;
	ekold = ek + 1.0;

	OneMinusecosE = 0; // Suppress the uninitialized warning.
	while(fabs(ek-ekold)>1.0E-14)
	{
		ekold = ek;
		OneMinusecosE = 1.0-eph.ecc*cos(ekold);
		ek = ek + (mk-ekold+eph.ecc*sin(ekold))/OneMinusecosE;
	}

	sek = sin(ek);
	cek = cos(ek);

	ekdot = eph.n/OneMinusecosE;

	relativistic = -4.442807633E-10*eph.ecc*eph.sqrta*sek;

	pk = atan2(eph.sq1e2*sek,cek-eph.ecc) + eph.aop;
	pkdot = eph.sq1e2*ekdot/OneMinusecosE;

	s2pk = sin(2.0*pk);
	c2pk = cos(2.0*pk);

	uk = pk + eph.cus*s2pk + eph.cuc*c2pk;
	suk = sin(uk);
	cuk = cos(uk);
	ukdot = pkdot*(1.0 + 2.0*(eph.cus*c2pk - eph.cuc*s2pk));

	rk = eph.A*OneMinusecosE + eph.crc*c2pk + eph.crs*s2pk;
	rkdot = eph.A*eph.ecc*sek*ekdot + 2.0*pkdot*(eph.crs*c2pk - eph.crc*s2pk);

	ik = eph.inc0 + eph.idot*tk + eph.cic*c2pk + eph.cis*s2pk;
	sik = sin(ik);
	cik = cos(ik);
	ikdot = eph.idot + 2.0*pkdot*(eph.cis*c2pk - eph.cic*s2pk);

	xpk = rk*cuk;
	ypk = rk*suk;
	xpkdot = rkdot*cuk - ypk*ukdot;
	ypkdot = rkdot*suk + xpk*ukdot;

	ok = eph.omg0 + tk*eph.omgkdot - OMEGA_EARTH*eph.toe.sec;
	sok = sin(ok);
	cok = cos(ok);

	pos[0] = xpk*cok - ypk*cik*sok;
	pos[1] = xpk*sok + ypk*cik*cok;
	pos[2] = ypk*sik;

	tmp = ypkdot*cik - ypk*sik*ikdot;

	vel[0] = -eph.omgkdot*pos[1] + xpkdot*cok - tmp*sok;
	vel[1] = eph.omgkdot*pos[0] + xpkdot*sok + tmp*cok;
	vel[2] = ypk*cik*ikdot + ypkdot*sik;

	// Satellite clock correction
	tk = g.sec - eph.toc.sec;

	if(tk>SECONDS_IN_HALF_WEEK)
		tk -= SECONDS_IN_WEEK;
	else if(tk<-SECONDS_IN_HALF_WEEK)
		tk += SECONDS_IN_WEEK;

	clk[0] = eph.af0 + tk*(eph.af1 + tk*eph.af2) + relativistic - eph.tgd;
	clk[1] = eph.af1 + 2.0*tk*eph.af2;

	return;
}

/*! \brief Compute Subframe from Ephemeris
 *  \param[in] eph Ephemeris of given SV
 *  \param[out] sbf Array of five sub-frames, 10 long words each
 */
void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, unsigned long sbf[5][N_DWRD_SBF])
{
	unsigned long wn;
	unsigned long toe;
	unsigned long toc;
	unsigned long iode;
	unsigned long iodc;
	long deltan;
	long cuc;
	long cus;
	long cic;
	long cis;
	long crc;
	long crs;
	unsigned long ecc;
	unsigned long sqrta;
	long m0;
	long omg0;
	long inc0;
	long aop;
	long omgdot;
	long idot;
	long af0;
	long af1;
	long af2;
	long tgd;
	int svhlth;
	int codeL2;

	unsigned long ura = 0UL;
	unsigned long dataId = 1UL;
	unsigned long sbf4_page25_svId = 63UL;
	unsigned long sbf5_page25_svId = 51UL;

	unsigned long wna;
	unsigned long toa;

	signed long alpha0,alpha1,alpha2,alpha3;
	signed long beta0,beta1,beta2,beta3;
	signed long A0,A1;
	signed long dtls;
	unsigned long tot,wnt,wnlsf,dtlsf,dn;
	unsigned long sbf4_page18_svId = 56UL;

	// FIXED: This has to be the "transmission" week number, not for the ephemeris reference time
	//wn = (unsigned long)(eph.toe.week%1024);
	wn = 0UL;
	toe = (unsigned long)(eph.toe.sec/16.0);
	toc = (unsigned long)(eph.toc.sec/16.0);
	iode = (unsigned long)(eph.iode);
	iodc = (unsigned long)(eph.iodc);
	deltan = (long)(eph.deltan/POW2_M43/PI);
	cuc = (long)(eph.cuc/POW2_M29);
	cus = (long)(eph.cus/POW2_M29);
	cic = (long)(eph.cic/POW2_M29);
	cis = (long)(eph.cis/POW2_M29);
	crc = (long)(eph.crc/POW2_M5);
	crs = (long)(eph.crs/POW2_M5);
	ecc = (unsigned long)(eph.ecc/POW2_M33);
	sqrta = (unsigned long)(eph.sqrta/POW2_M19);
	m0 = (long)(eph.m0/POW2_M31/PI);
	omg0 = (long)(eph.omg0/POW2_M31/PI);
	inc0 = (long)(eph.inc0/POW2_M31/PI);
	aop = (long)(eph.aop/POW2_M31/PI);
	omgdot = (long)(eph.omgdot/POW2_M43/PI);
	idot = (long)(eph.idot/POW2_M43/PI);
	af0 = (long)(eph.af0/POW2_M31);
	af1 = (long)(eph.af1/POW2_M43);
	af2 = (long)(eph.af2/POW2_M55);
	tgd = (long)(eph.tgd/POW2_M31);
	svhlth = (unsigned long)(eph.svhlth);
	codeL2 = (unsigned long)(eph.codeL2);

	wna = (unsigned long)(eph.toe.week%256);
	toa = (unsigned long)(eph.toe.sec/4096.0);

	alpha0 = (signed long)round(ionoutc.alpha0/POW2_M30);
	alpha1 = (signed long)round(ionoutc.alpha1/POW2_M27);
	alpha2 = (signed long)round(ionoutc.alpha2/POW2_M24);
	alpha3 = (signed long)round(ionoutc.alpha3/POW2_M24);
	beta0 = (signed long)round(ionoutc.beta0/2048.0);
	beta1 = (signed long)round(ionoutc.beta1/16384.0);
	beta2 = (signed long)round(ionoutc.beta2/65536.0);
	beta3 = (signed long)round(ionoutc.beta3/65536.0);
	A0 = (signed long)round(ionoutc.A0/POW2_M30);
	A1 = (signed long)round(ionoutc.A1/POW2_M50);
	dtls = (signed long)(ionoutc.dtls);
	tot = (unsigned long)(ionoutc.tot/4096);
	wnt = (unsigned long)(ionoutc.wnt%256);

	// 2016/12/31 (Sat) -> WNlsf = 1929, DN = 7 (http://navigationservices.agi.com/GNSSWeb/)
	// Days are counted from 1 to 7 (Sunday is 1).
	if (ionoutc.leapen==TRUE)
	{
		wnlsf = (unsigned long)(ionoutc.wnlsf%256);
		dn = (unsigned long)(ionoutc.dn);
		dtlsf = (unsigned long)(ionoutc.dtlsf);
	}
	else
	{
		wnlsf = 1929%256;
		dn = 7;
		dtlsf = 18;
	}
	// Subframe 1
	sbf[0][0] = 0x8B0000UL<<6;
	sbf[0][1] = 0x1UL<<8;
	sbf[0][2] = ((wn&0x3FFUL)<<20) | ((codeL2&0x3UL)<<18) | ((ura&0xFUL)<<14) | ((svhlth&0x3FUL)<<8) | (((iodc>>8)&0x3UL)<<6);
	sbf[0][3] = 0UL;
	sbf[0][4] = 0UL;
	sbf[0][5] = 0UL;
	sbf[0][6] = (tgd&0xFFUL)<<6;
	sbf[0][7] = ((iodc&0xFFUL)<<22) | ((toc&0xFFFFUL)<<6);
	sbf[0][8] = ((af2&0xFFUL)<<22) | ((af1&0xFFFFUL)<<6);
	sbf[0][9] = (af0&0x3FFFFFUL)<<8;

	// Subframe 2
	sbf[1][0] = 0x8B0000UL<<6;
	sbf[1][1] = 0x2UL<<8;
	sbf[1][2] = ((iode&0xFFUL)<<22) | ((crs&0xFFFFUL)<<6);
	sbf[1][3] = ((deltan&0xFFFFUL)<<14) | (((m0>>24)&0xFFUL)<<6);
	sbf[1][4] = (m0&0xFFFFFFUL)<<6;
	sbf[1][5] = ((cuc&0xFFFFUL)<<14) | (((ecc>>24)&0xFFUL)<<6);
	sbf[1][6] = (ecc&0xFFFFFFUL)<<6;
	sbf[1][7] = ((cus&0xFFFFUL)<<14) | (((sqrta>>24)&0xFFUL)<<6);
	sbf[1][8] = (sqrta&0xFFFFFFUL)<<6;
	sbf[1][9] = (toe&0xFFFFUL)<<14;

	// Subframe 3
	sbf[2][0] = 0x8B0000UL<<6;
	sbf[2][1] = 0x3UL<<8;
	sbf[2][2] = ((cic&0xFFFFUL)<<14) | (((omg0>>24)&0xFFUL)<<6);
	sbf[2][3] = (omg0&0xFFFFFFUL)<<6;
	sbf[2][4] = ((cis&0xFFFFUL)<<14) | (((inc0>>24)&0xFFUL)<<6);
	sbf[2][5] = (inc0&0xFFFFFFUL)<<6;
	sbf[2][6] = ((crc&0xFFFFUL)<<14) | (((aop>>24)&0xFFUL)<<6);
	sbf[2][7] = (aop&0xFFFFFFUL)<<6;
	sbf[2][8] = (omgdot&0xFFFFFFUL)<<6;
	sbf[2][9] = ((iode&0xFFUL)<<22) | ((idot&0x3FFFUL)<<8);

	if (ionoutc.vflg==TRUE)
	{
		// Subframe 4, page 18
		sbf[3][0] = 0x8B0000UL<<6;
		sbf[3][1] = 0x4UL<<8;
		sbf[3][2] = (dataId<<28) | (sbf4_page18_svId<<22) | ((alpha0&0xFFUL)<<14) | ((alpha1&0xFFUL)<<6);
		sbf[3][3] = ((alpha2&0xFFUL)<<22) | ((alpha3&0xFFUL)<<14) | ((beta0&0xFFUL)<<6);
		sbf[3][4] = ((beta1&0xFFUL)<<22) | ((beta2&0xFFUL)<<14) | ((beta3&0xFFUL)<<6);
		sbf[3][5] = (A1&0xFFFFFFUL)<<6;
		sbf[3][6] = ((A0>>8)&0xFFFFFFUL)<<6;
		sbf[3][7] = ((A0&0xFFUL)<<22) | ((tot&0xFFUL)<<14) | ((wnt&0xFFUL)<<6);
		sbf[3][8] = ((dtls&0xFFUL)<<22) | ((wnlsf&0xFFUL)<<14) | ((dn&0xFFUL)<<6);
		sbf[3][9] = (dtlsf&0xFFUL)<<22;

	}
	else
	{
		// Subframe 4, page 25
		sbf[3][0] = 0x8B0000UL<<6;
		sbf[3][1] = 0x4UL<<8;
		sbf[3][2] = (dataId<<28) | (sbf4_page25_svId<<22);
		sbf[3][3] = 0UL;
		sbf[3][4] = 0UL;
		sbf[3][5] = 0UL;
		sbf[3][6] = 0UL;
		sbf[3][7] = 0UL;
		sbf[3][8] = 0UL;
		sbf[3][9] = 0UL;
	}

	// Subframe 5, page 25
	sbf[4][0] = 0x8B0000UL<<6;
	sbf[4][1] = 0x5UL<<8;
	sbf[4][2] = (dataId<<28) | (sbf5_page25_svId<<22) | ((toa&0xFFUL)<<14) | ((wna&0xFFUL)<<6);
	sbf[4][3] = 0UL;
	sbf[4][4] = 0UL;
	sbf[4][5] = 0UL;
	sbf[4][6] = 0UL;
	sbf[4][7] = 0UL;
	sbf[4][8] = 0UL;
	sbf[4][9] = 0UL;

	return;
}

/*! \brief Count number of bits set to 1
 *  \param[in] v long word in which bits are counted
 *  \returns Count of bits set to 1
 */
unsigned long countBits(unsigned long v)
{
	unsigned long c;
	const int S[] = {1, 2, 4, 8, 16};
	const unsigned long B[] = {
		0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF};

	c = v;
	c = ((c >> S[0]) & B[0]) + (c & B[0]);
	c = ((c >> S[1]) & B[1]) + (c & B[1]);
	c = ((c >> S[2]) & B[2]) + (c & B[2]);
	c = ((c >> S[3]) & B[3]) + (c & B[3]);
	c = ((c >> S[4]) & B[4]) + (c & B[4]);

	return(c);
}

/*! \brief Compute the Checksum for one given word of a subframe
 *  \param[in] source The input data
 *  \param[in] nib Does this word contain non-information-bearing bits?
 *  \returns Computed Checksum
 */
unsigned long computeChecksum(unsigned long source, int nib)
{
	/*
	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
	Bits 29 to  6 = Source data bits, d1, d2, ..., d24
	Bits  5 to  0 = Empty parity bits
	*/

	/*
	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
	Bits 29 to  6 = Data bits transmitted by the SV, D1, D2, ..., D24
	Bits  5 to  0 = Computed parity bits, D25, D26, ..., D30
	*/

	/*
	                  1            2           3
	bit    12 3456 7890 1234 5678 9012 3456 7890
	---    -------------------------------------
	D25    11 1011 0001 1111 0011 0100 1000 0000
	D26    01 1101 1000 1111 1001 1010 0100 0000
	D27    10 1110 1100 0111 1100 1101 0000 0000
	D28    01 0111 0110 0011 1110 0110 1000 0000
	D29    10 1011 1011 0001 1111 0011 0100 0000
	D30    00 1011 0111 1010 1000 1001 1100 0000
	*/

	unsigned long bmask[6] = {
		0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
		0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL };

	unsigned long D;
	unsigned long d = source & 0x3FFFFFC0UL;
	unsigned long D29 = (source>>31)&0x1UL;
	unsigned long D30 = (source>>30)&0x1UL;

	if (nib) // Non-information bearing bits for word 2 and 10
	{
		/*
		Solve bits 23 and 24 to preserve parity check
		with zeros in bits 29 and 30.
		*/

		if ((D30 + countBits(bmask[4] & d)) % 2)
			d ^= (0x1UL<<6);
		if ((D29 + countBits(bmask[5] & d)) % 2)
			d ^= (0x1UL<<7);
	}

	D = d;
	if (D30)
		D ^= 0x3FFFFFC0UL;

	D |= ((D29 + countBits(bmask[0] & d)) % 2) << 5;
	D |= ((D30 + countBits(bmask[1] & d)) % 2) << 4;
	D |= ((D29 + countBits(bmask[2] & d)) % 2) << 3;
	D |= ((D30 + countBits(bmask[3] & d)) % 2) << 2;
	D |= ((D30 + countBits(bmask[4] & d)) % 2) << 1;
	D |= ((D29 + countBits(bmask[5] & d)) % 2);

	D &= 0x3FFFFFFFUL;
	//D |= (source & 0xC0000000UL); // Add D29* and D30* from source data bits

	return(D);
}

/*! \brief Replace all 'E' exponential designators to 'D'
 *  \param str String in which all occurrences of 'E' are replaced with *  'D'
 *  \param len Length of input string in bytes
 *  \returns Number of characters replaced
 */
int replaceExpDesignator(char *str, int len)
{
	int i,n=0;

	for (i=0; i<len; i++)
	{
		if (str[i]=='D')
		{
			n++;
			str[i] = 'E';
		}
	}

	return(n);
}

//Tells the number of seconds in g1 and g0, in the code below gmax and gmin are placed here
//So for that is calculates the number of seconds between gmax and gmin
double subGpsTime(gpstime_t g1, gpstime_t g0)
{
	double dt;

	dt = g1.sec - g0.sec;
	dt += (double)(g1.week - g0.week) * SECONDS_IN_WEEK;

	return(dt);
}

//Avoid Rounding Error (dt is seconds that u need to add, basically this function is used to increase the gps time)
gpstime_t incGpsTime(gpstime_t g0, double dt)
{
	gpstime_t g1;

	g1.week = g0.week;
	g1.sec = g0.sec + dt;

	g1.sec = round(g1.sec*1000.0)/1000.0; // Avoid rounding error

	while (g1.sec>=SECONDS_IN_WEEK)
	{
		g1.sec -= SECONDS_IN_WEEK;
		g1.week++;
	}

	while (g1.sec<0.0)
	{
		g1.sec += SECONDS_IN_WEEK;
		g1.week--;
	}

	return(g1);
}

/*! \brief Read Ephemeris data from the RINEX Navigation file */
/*  \param[out] eph Array of Output SV ephemeris data
 *  \param[in] fname File name of the RINEX file
 *  \returns Number of sets of ephemerides in the file
 */
int readRinexNavAll(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname)
{
	FILE *fp;
	int ieph;

	int sv;
	char str[MAX_CHAR];
	char tmp[20];

	datetime_t t;
	gpstime_t g;
	gpstime_t g0;
	double dt;

	int flags = 0x0;

	if (NULL==(fp=fopen(fname, "rt")))
		return(-1);

	// Clear valid flag
	for (ieph=0; ieph<EPHEM_ARRAY_SIZE; ieph++)
		for (sv=0; sv<MAX_SAT; sv++)
			eph[ieph][sv].vflg = 0;

	// Read header lines
	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		if (strncmp(str+60, "END OF HEADER", 13)==0)
			break;
		else if (strncmp(str+60, "ION ALPHA", 9)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha3 = atof(tmp);

			//read wntlsf, dn, and dtlsf from fil

			flags |= 0x1;
		}
		else if (strncmp(str+60, "ION BETA", 8)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta3 = atof(tmp);

			flags |= 0x1<<1;
		}
		else if (strncmp(str+60, "DELTA-UTC", 9)==0)
		{
			strncpy(tmp, str+3, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A0 = atof(tmp);

			strncpy(tmp, str+22, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A1 = atof(tmp);

			strncpy(tmp, str+41, 9);
			tmp[9] = 0;
			ionoutc->tot = atoi(tmp);

			strncpy(tmp, str+50, 9);
			tmp[9] = 0;
			ionoutc->wnt = atoi(tmp);

			if (ionoutc->tot%4096==0)
				flags |= 0x1<<2;
		}
		else if (strncmp(str+60, "LEAP SECONDS", 12)==0)
		{
			strncpy(tmp, str, 6);
			tmp[6] = 0;
			ionoutc->dtls = atoi(tmp);

			flags |= 0x1<<3;
		}
	}

	ionoutc->vflg = FALSE;
	if (flags==0xF) // Read all Iono/UTC lines
		ionoutc->vflg = TRUE;

	// Read ephemeris blocks
	g0.week = -1;
	ieph = 0;

	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		// PRN [CAN KNOW WHERE PRN EXISTS FOR A GPS EPHEMERIS FILE, CAN FIND PRN FOR GLONASS, GALILIEO, AND BEIDOU SIMILARLY]
		strncpy(tmp, str, 2);
		tmp[2] = 0;
		sv = atoi(tmp)-1;

		// EPOCH
		strncpy(tmp, str+3, 2);
		tmp[2] = 0;
		t.y = atoi(tmp) + 2000; //YEAR

		strncpy(tmp, str+6, 2);
		tmp[2] = 0;
		t.m = atoi(tmp); //MONTH

		strncpy(tmp, str+9, 2);
		tmp[2] = 0;
		t.d = atoi(tmp); //DAY

		strncpy(tmp, str+12, 2);
		tmp[2] = 0;
		t.hh = atoi(tmp); //HOUR

		strncpy(tmp, str+15, 2);
		tmp[2] = 0;
		t.mm = atoi(tmp); //MIN

		strncpy(tmp, str+18, 4);
		tmp[2] = 0;
		t.sec = atof(tmp); //SEC

		date2gps(&t, &g);

		if (g0.week==-1)
			g0 = g;

		// Check current time of clock
		dt = subGpsTime(g, g0);

		if (dt>SECONDS_IN_HOUR)
		{
			g0 = g;
			ieph++; // a new set of ephemerides

			if (ieph>=EPHEM_ARRAY_SIZE)
				break;
		}

		// Date and time
		eph[ieph][sv].t = t;

		// SV CLK
		eph[ieph][sv].toc = g;

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19); // tmp[15]='E';
		eph[ieph][sv].af0 = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af1 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af2 = atof(tmp);

		// BROADCAST ORBIT - 1
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iode = (int)atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crs = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].deltan = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].m0 = atof(tmp);

		// BROADCAST ORBIT - 2
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cuc = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].ecc = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cus = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].sqrta = atof(tmp);

		// BROADCAST ORBIT - 3
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.sec = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cic = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omg0 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cis = atof(tmp);

		// BROADCAST ORBIT - 4
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].inc0 = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crc = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].aop = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omgdot = atof(tmp);

		// BROADCAST ORBIT - 5
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].idot = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].codeL2 = (int)atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.week = (int)atof(tmp);

		// BROADCAST ORBIT - 6
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].svhlth = (int)atof(tmp);
		if ((eph[ieph][sv].svhlth>0) && (eph[ieph][sv].svhlth<32))
			eph[ieph][sv].svhlth += 32; // Set MSB to 1

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].tgd = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iodc = (int)atof(tmp);

		// BROADCAST ORBIT - 7
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		// Set valid flag
		eph[ieph][sv].vflg = 1;

		// Update the working variables
		eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
		eph[ieph][sv].n = sqrt(GM_EARTH/(eph[ieph][sv].A*eph[ieph][sv].A*eph[ieph][sv].A)) + eph[ieph][sv].deltan;
		eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc*eph[ieph][sv].ecc);
		eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - OMEGA_EARTH;
	}

	fclose(fp);

	if (g0.week>=0)
		ieph += 1; // Number of sets of ephemerides

	return(ieph);
}

double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh, double *azel)
{
	double iono_delay = 0.0;
	double E,phi_u,lam_u,F;

	if (ionoutc->enable==FALSE)
		return (0.0); // No ionospheric delay

	E = azel[1]/PI;
	phi_u = llh[0]/PI;
	lam_u = llh[1]/PI;

	// Obliquity factor
	F = 1.0 + 16.0*pow((0.53 - E),3.0);

	if (ionoutc->vflg==FALSE)
		iono_delay = F*5.0e-9*SPEED_OF_LIGHT;
	else
	{
		double t,psi,phi_i,lam_i,phi_m,phi_m2,phi_m3;
		double AMP,PER,X,X2,X4;

		// Earth's central angle between the user position and the earth projection of
		// ionospheric intersection point (semi-circles)
		psi = 0.0137/(E + 0.11) - 0.022;

		// Geodetic latitude of the earth projection of the ionospheric intersection point
		// (semi-circles)
		phi_i = phi_u + psi*cos(azel[0]);
		if(phi_i>0.416)
			phi_i = 0.416;
		else if(phi_i<-0.416)
			phi_i = -0.416;

		// Geodetic longitude of the earth projection of the ionospheric intersection point
		// (semi-circles)
		lam_i = lam_u + psi*sin(azel[0])/cos(phi_i*PI);

		// Geomagnetic latitude of the earth projection of the ionospheric intersection
		// point (mean ionospheric height assumed 350 km) (semi-circles)
		phi_m = phi_i + 0.064*cos((lam_i - 1.617)*PI);
		phi_m2 = phi_m*phi_m;
		phi_m3 = phi_m2*phi_m;

		AMP = ionoutc->alpha0 + ionoutc->alpha1*phi_m
			+ ionoutc->alpha2*phi_m2 + ionoutc->alpha3*phi_m3;
		if (AMP<0.0)
			AMP = 0.0;

		PER = ionoutc->beta0 + ionoutc->beta1*phi_m
			+ ionoutc->beta2*phi_m2 + ionoutc->beta3*phi_m3;
		if (PER<72000.0)
			PER = 72000.0;

		// Local time (sec)
		t = SECONDS_IN_DAY/2.0*lam_i + g.sec;
		while(t>=SECONDS_IN_DAY)
			t -= SECONDS_IN_DAY;
		while(t<0)
			t += SECONDS_IN_DAY;

		// Phase (radians)
		X = 2.0*PI*(t - 50400.0)/PER;

		if(fabs(X)<1.57)
		{
			X2 = X*X;
			X4 = X2*X2;
			iono_delay = F*(5.0e-9 + AMP*(1.0 - X2/2.0 + X4/24.0))*SPEED_OF_LIGHT;
		}
		else
			iono_delay = F*5.0e-9*SPEED_OF_LIGHT*(ionoChange/100);
	}

	return (iono_delay);
}

/*! \brief Compute range between a satellite and the receiver
 *  \param[out] rho The computed range
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at time of receiving the signal
 *  \param[in] xyz position of the receiver
 */
void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g, double xyz[])
{
	double pos[3],vel[3],clk[2];
	double los[3];
	double tau;
	double range,rate;
	double xrot,yrot;

	double llh[3],neu[3];
	double tmat[3][3];

	// SV position at time of the pseudorange observation.
	satpos(eph, g, pos, vel, clk);

	// Receiver to satellite vector and light-time.
	subVect(los, pos, xyz);
	tau = normVect(los)/SPEED_OF_LIGHT;

	// Extrapolate the satellite position backwards to the transmission time.
	pos[0] -= vel[0]*tau;
	pos[1] -= vel[1]*tau;
	pos[2] -= vel[2]*tau;

	// Earth rotation correction. The change in velocity can be neglected.
	xrot = pos[0] + pos[1]*OMEGA_EARTH*tau;
	yrot = pos[1] - pos[0]*OMEGA_EARTH*tau;
	pos[0] = xrot;
	pos[1] = yrot;

	// New observer to satellite vector and satellite range.
	subVect(los, pos, xyz);
	range = normVect(los);
	rho->d = range;

	// Pseudorange.
	rho->range = range - SPEED_OF_LIGHT*clk[0];

	// Relative velocity of SV and receiver.
	rate = dotProd(vel, los)/range;

	// Pseudorange rate.
	rho->rate = rate; // - SPEED_OF_LIGHT*clk[1];

	// Time of application.
	rho->g = g;

	// Azimuth and elevation angles.
	xyz2llh(xyz, llh);
	ltcmat(llh, tmat);
	ecef2neu(los, tmat, neu);
	neu2azel(rho->azel, neu);

	// Add ionospheric delay
	rho->iono_delay = ionosphericDelay(ionoutc, g, llh, rho->azel);
	rho->range += rho->iono_delay;

	return;
}

/*! \brief Compute the code phase for a given channel (satellite)
 *  \param chan Channel on which we operate (is updated)
 *  \param[in] rho1 Current range, after \a dt has expired
 *  \param[in dt delta-t (time difference) in seconds
 */
void computeCodePhase(channel_t *chan, range_t rho1, double dt)
{
	double ms;
	int ims;
	double rhorate;

	// Pseudorange rate.
	rhorate = (rho1.range - chan->rho0.range)/dt;

	// Carrier and code frequency.
	chan->f_carr = -rhorate/LAMBDA_L1;
	chan->f_code = CODE_FREQ + chan->f_carr*CARR_TO_CODE;

	// Initial code phase and data bit counters.
	ms = ((subGpsTime(chan->rho0.g,chan->g0)+6.0) - chan->rho0.range/SPEED_OF_LIGHT)*1000.0;

	ims = (int)ms;
	chan->code_phase = (ms-(double)ims)*CA_SEQ_LEN; // in chip

	chan->iword = ims/600; // 1 word = 30 bits = 600 ms
	ims -= chan->iword*600;

	chan->ibit = ims/20; // 1 bit = 20 code = 20 ms
	ims -= chan->ibit*20;

	chan->icode = ims; // 1 code = 1 ms

	chan->codeCA = chan->ca[(int)chan->code_phase]*2-1;
	chan->dataBit = (int)((chan->dwrd[chan->iword]>>(29-chan->ibit)) & 0x1UL)*2-1;

	// Save current pseudorange
	chan->rho0 = rho1;

	return;
}

/*! \brief Read the list of user motions from the input file
 *  \param[out] xyz Output array of ECEF vectors for user motion
 *  \param[[in] filename File name of the text input file
 *  \returns Number of user data motion records read, -1 on error
 */
int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename)
{
	FILE *fp;
	int numd;
	char str[MAX_CHAR];
	double t,x,y,z;

	if (NULL==(fp=fopen(filename,"rt")))
		return(-1);

	for (numd=0; numd<USER_MOTION_SIZE; numd++)
	{
		if (fgets(str, MAX_CHAR, fp)==NULL)
			break;

		if (EOF==sscanf(str, "%lf,%lf,%lf,%lf", &t, &x, &y, &z)) // Read CSV line
			break;

		xyz[numd][0] = x;
		xyz[numd][1] = y;
		xyz[numd][2] = z;
	}

	fclose(fp);

	return (numd);
}

/*! \brief Read the list of user motions from the input file
 *  \param[out] xyz Output array of LatLonHei coordinates for user motion
 *  \param[[in] filename File name of the text input file with format Lat,Lon,Hei
 *  \returns Number of user data motion records read, -1 on error
 *
 * Added by romalvarezllorens@gmail.com
 */
int readUserMotionLLH(double xyz[USER_MOTION_SIZE][3], const char *filename)
{
	FILE *fp;
	int numd;
	double t,llh[3];
	char str[MAX_CHAR];

	if (NULL==(fp=fopen(filename,"rt")))
		return(-1);

	for (numd=0; numd<USER_MOTION_SIZE; numd++)
	{
		if (fgets(str, MAX_CHAR, fp)==NULL)
			break;

		if (EOF==sscanf(str, "%lf,%lf,%lf,%lf", &t, &llh[0], &llh[1], &llh[2])) // Read CSV line
			break;

		if (llh[0] > 90.0 || llh[0] < -90.0 || llh[1]>180.0 || llh[1] < -180.0)
		{
			fprintf(stderr, "ERROR: Invalid file format (time[s], latitude[deg], longitude[deg], height [m].\n");
			numd = 0; // Empty user motion
			break;
		}

		llh[0] /= R2D; // convert to RAD
		llh[1] /= R2D; // convert to RAD

		llh2xyz(llh, xyz[numd]);
	}

	fclose(fp);

	return (numd);
}

int readNmeaGGA(double xyz[USER_MOTION_SIZE][3], const char *filename)
{
	FILE *fp;
	int numd = 0;
	char str[MAX_CHAR];
	char *token;
	double llh[3],pos[3];
	char tmp[8];

	if (NULL==(fp=fopen(filename,"rt")))
		return(-1);

	while (1)
	{
		if (fgets(str, MAX_CHAR, fp)==NULL)
			break;

		token = strtok(str, ",");

		if (strncmp(token+3, "GGA", 3)==0)
		{
			token = strtok(NULL, ","); // Date and time

			token = strtok(NULL, ","); // Latitude
			strncpy(tmp, token, 2);
			tmp[2] = 0;

			llh[0] = atof(tmp) + atof(token+2)/60.0;

			token = strtok(NULL, ","); // North or south
			if (token[0]=='S')
				llh[0] *= -1.0;

			llh[0] /= R2D; // in radian

			token = strtok(NULL, ","); // Longitude
			strncpy(tmp, token, 3);
			tmp[3] = 0;

			llh[1] = atof(tmp) + atof(token+3)/60.0;

			token = strtok(NULL, ","); // East or west
			if (token[0]=='W')
				llh[1] *= -1.0;

			llh[1] /= R2D; // in radian

			token = strtok(NULL, ","); // GPS fix
			token = strtok(NULL, ","); // Number of satellites
			token = strtok(NULL, ","); // HDOP

			token = strtok(NULL, ","); // Altitude above meas sea level

			llh[2] = atof(token);

			token = strtok(NULL, ","); // in meter

			token = strtok(NULL, ","); // Geoid height above WGS84 ellipsoid

			llh[2] += atof(token);

			// Convert geodetic position into ECEF coordinates
			llh2xyz(llh, pos);

			xyz[numd][0] = pos[0];
			xyz[numd][1] = pos[1];
			xyz[numd][2] = pos[2];

			// Update the number of track points
			numd++;

			if (numd>=USER_MOTION_SIZE)
				break;
		}
	}

	fclose(fp);

	return (numd);
}

int generateNavMsg(gpstime_t g, channel_t *chan, int init)
{
	int iwrd,isbf;
	gpstime_t g0;
	unsigned long wn,tow;
	unsigned sbfwrd;
	unsigned long prevwrd;
	int nib;

	g0.week = g.week;
	g0.sec = (double)(((unsigned long)(g.sec+0.5))/30UL) * 30.0; // Align with the full frame length = 30 sec
	chan->g0 = g0; // Data bit reference time

	wn = (unsigned long)(g0.week%1024);
	tow = ((unsigned long)g0.sec)/6UL;

	if (init==1) // Initialize subframe 5
	{
		prevwrd = 0UL;

		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
		{
			sbfwrd = chan->sbf[4][iwrd];

			// Add TOW-count message into HOW
			if (iwrd==1)
				sbfwrd |= ((tow&0x1FFFFUL)<<13);

			// Compute checksum
			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
			chan->dwrd[iwrd] = computeChecksum(sbfwrd, nib);

			prevwrd = chan->dwrd[iwrd];
		}
	}
	else // Save subframe 5
	{
		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
		{
			chan->dwrd[iwrd] = chan->dwrd[N_DWRD_SBF*N_SBF+iwrd];

			prevwrd = chan->dwrd[iwrd];
		}
		/*
		// Sanity check
		if (((chan->dwrd[1])&(0x1FFFFUL<<13)) != ((tow&0x1FFFFUL)<<13))
		{
			fprintf(stderr, "\nWARNING: Invalid TOW in subframe 5.\n");
			return(0);
		}
		*/
	}

	for (isbf=0; isbf<N_SBF; isbf++)
	{
		tow++;

		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
		{
			sbfwrd = chan->sbf[isbf][iwrd];

			// Add transmission week number to Subframe 1
			if ((isbf==0)&&(iwrd==2))
				sbfwrd |= (wn&0x3FFUL)<<20;

			// Add TOW-count message into HOW
			if (iwrd==1)
				sbfwrd |= ((tow&0x1FFFFUL)<<13);

			// Compute checksum
			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
			chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd] = computeChecksum(sbfwrd, nib);

			prevwrd = chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd];
		}
	}

	return(1);
}

int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask, double *azel)
{
	double llh[3],neu[3];
	double pos[3],vel[3],clk[3],los[3];
	double tmat[3][3];

	if (eph.vflg != 1)
		return (-1); // Invalid

	xyz2llh(xyz,llh);
	ltcmat(llh, tmat);

	satpos(eph, g, pos, vel, clk);
	subVect(los, pos, xyz);
	ecef2neu(los, tmat, neu);
	neu2azel(azel, neu);

	if (azel[1]*R2D > elvMask)
		return (1); // Visible
	// else
	return (0); // Invisible
}

int allocateChannel(channel_t *chan, ephem_t *eph, ionoutc_t ionoutc, gpstime_t grx, double *xyz, double elvMask)
{
	int nsat=0;
	int i,sv;
	double azel[2];

	range_t rho;
	double ref[3]={0.0};
	double r_ref,r_xyz;
	double phase_ini;
	
	switch (pathLossMode)
	{
	case 0: //ADDED BY ME		[DISBALE MODE:	Allocates Satellites in for loop, All Channels are set at Max Gain Setting]
		for (sv=0; sv<MAX_SAT; sv++)
		{
			if(checkSatVisibility(eph[sv], grx, xyz, 0.0, azel)==1)
			{
				nsat++; // Number of visible satellites

				if (allocatedSat[sv]==-1) // Visible but not allocated
				{
					// Allocated new satellite
					for (i=0; i<MAX_CHAN; i++)
					{
						if (chan[i].prn==0)
						{
							// Initialize channel
							chan[i].prn = sv+1;
							chan[i].azel[0] = azel[0];
							chan[i].azel[1] = azel[1];

							// C/A code generation
							codegen(chan[i].ca, chan[i].prn);

							// Generate subframe
							eph2sbf(eph[sv], ionoutc, chan[i].sbf);

							// Generate navigation message
							generateNavMsg(grx, &chan[i], 1);

							// Initialize pseudorange
							computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
							chan[i].rho0 = rho;

							// Initialize carrier phase
							r_xyz = rho.range;

							computeRange(&rho, eph[sv], &ionoutc, grx, ref);
							r_ref = rho.range;

							phase_ini = 0.0; // TODO: Must initialize properly
							//phase_ini = (2.0*r_ref - r_xyz)/LAMBDA_L1;
#ifdef FLOAT_CARR_PHASE
							chan[i].carr_phase = phase_ini - floor(phase_ini);
#else
							phase_ini -= floor(phase_ini);
							chan[i].carr_phase = (unsigned int)(512.0 * 65536.0 * phase_ini);
#endif
							// Done.
							break;
						}
					}

					// Set satellite allocation channel
					if (i<MAX_CHAN)
						allocatedSat[sv] = i;
				}
			}
			else if (allocatedSat[sv]>=0) // Not visible but allocated
			{
				// Clear channel
				chan[allocatedSat[sv]].prn = 0;

				// Clear satellite allocation flag
				allocatedSat[sv] = -1;
			}
		}
		break;
	case 1: //ADDED BY ME		[AUTO MODE:	Allocates Satellites in for loop, Channel Gain is set Automatically]
		for (sv=0; sv<MAX_SAT; sv++)
		{
			if(checkSatVisibility(eph[sv], grx, xyz, 0.0, azel)==1)
			{
				nsat++; // Number of visible satellites

				if (allocatedSat[sv]==-1) // Visible but not allocated
				{
					// Allocated new satellite
					for (i=0; i<MAX_CHAN; i++)
					{
						if (chan[i].prn==0)
						{
							// Initialize channel
							chan[i].prn = sv+1;
							chan[i].azel[0] = azel[0];
							chan[i].azel[1] = azel[1];

							// C/A code generation
							codegen(chan[i].ca, chan[i].prn);

							// Generate subframe
							eph2sbf(eph[sv], ionoutc, chan[i].sbf);

							// Generate navigation message
							generateNavMsg(grx, &chan[i], 1);

							// Initialize pseudorange
							computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
							chan[i].rho0 = rho;

							// Initialize carrier phase
							r_xyz = rho.range;

							computeRange(&rho, eph[sv], &ionoutc, grx, ref);
							r_ref = rho.range;

							phase_ini = 0.0; // TODO: Must initialize properly
							//phase_ini = (2.0*r_ref - r_xyz)/LAMBDA_L1;
#ifdef FLOAT_CARR_PHASE
							chan[i].carr_phase = phase_ini - floor(phase_ini);
#else
							phase_ini -= floor(phase_ini);
							chan[i].carr_phase = (unsigned int)(512.0 * 65536.0 * phase_ini);
#endif
							// Done.
							break;
						}
					}

					// Set satellite allocation channel
					if (i<MAX_CHAN)
						allocatedSat[sv] = i;
				}
			}
			else if (allocatedSat[sv]>=0) // Not visible but allocated
			{
				// Clear channel
				chan[allocatedSat[sv]].prn = 0;

				// Clear satellite allocation flag
				allocatedSat[sv] = -1;
			}
		}
		break;
	case 2: //ADDED BY ME		[FIX MODE:	Allocates Satellites Defined by User Using chanIDAr array, Gain is set at a user-defined fixed value]
		for (i = 0; i < MAX_CHAN; i++)
		{
			if(chanModeAr[i] == 1)
			{
				sv = chanIDAr[i] - 1;
				if(checkSatVisibility(eph[sv], grx, xyz, 0.0, azel)==1)
				{
					nsat++;
					if (allocatedSat[sv]==-1)
					{
						if(chan[i].prn == 0)
						{
							chan[i].prn = sv+1;
							chan[i].azel[0] = azel[0];
							chan[i].azel[1] = azel[1];
							
							codegen(chan[i].ca, chan[i].prn);
							eph2sbf(eph[sv], ionoutc, chan[i].sbf);
							generateNavMsg(grx, &chan[i], 1);
							
							computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
							chan[i].rho0 = rho;
							
							r_xyz = rho.range;
							computeRange(&rho, eph[sv], &ionoutc, grx, ref);
							r_ref = rho.range;
							
							phase_ini = 0.0;
#ifdef FLOAT_CARR_PHASE		
							chan[i].carr_phase = phase_ini - floor(phase_ini);
#else		
							phase_ini -= floor(phase_ini);
							chan[i].carr_phase = (unsigned int)(512.0 * 65536.0 * phase_ini);
#endif			
							//break;
						}
						if (i<MAX_CHAN)
							allocatedSat[sv] = i;
					}
				}
				else if (allocatedSat[sv]>=0)
				{
					chan[allocatedSat[sv]].prn = 0;
					allocatedSat[sv] = -1;
				}
			}
		}
		break;
	case 3: //ADDED BY ME		[VAR MODE:	Allocates Satellites Defined by User Using chanIDAr array, Gain is set at user defined values given by chanStrAr]
		for (i = 0; i < MAX_CHAN; i++)
		{
			if(chanModeAr[i] == 1)
			{
				sv = chanIDAr[i] - 1;
				if(checkSatVisibility(eph[sv], grx, xyz, 0.0, azel)==1)
				{
					nsat++;
					if (allocatedSat[sv]==-1)
					{
						if(chan[i].prn == 0)
						{
							chan[i].prn = sv+1;
							chan[i].azel[0] = azel[0];
							chan[i].azel[1] = azel[1];
							
							codegen(chan[i].ca, chan[i].prn);
							eph2sbf(eph[sv], ionoutc, chan[i].sbf);
							generateNavMsg(grx, &chan[i], 1);
							
							computeRange(&rho, eph[sv], &ionoutc, grx, xyz);
							chan[i].rho0 = rho;
							
							r_xyz = rho.range;
							computeRange(&rho, eph[sv], &ionoutc, grx, ref);
							r_ref = rho.range;
							
							phase_ini = 0.0;
#ifdef FLOAT_CARR_PHASE		
							chan[i].carr_phase = phase_ini - floor(phase_ini);
#else		
							phase_ini -= floor(phase_ini);
							chan[i].carr_phase = (unsigned int)(512.0 * 65536.0 * phase_ini);
#endif			
							//break;
						}
						if (i<MAX_CHAN)
							allocatedSat[sv] = i;
					}
				}
				else if (allocatedSat[sv]>=0)
				{
					chan[allocatedSat[sv]].prn = 0;
					allocatedSat[sv] = -1;
				}
			}
		}
		break;
	default:
		break;
	}
	return(nsat);
}

void usage(void)
{
	fprintf(stderr, "Usage: gps-sdr-sim [options]\n"
		"Options:\n"
		"  -e <gps_nav>     	RINEX navigation file for GPS ephemerides (required)\n"
		"  -u <user_motion> 	User motion file in ECEF x, y, z format (dynamic mode)\n"
		"  -x <user_motion> 	User motion file in lat, lon, height format (dynamic mode)\n"
		"  -g <nmea_gga>    	NMEA GGA stream (dynamic mode)\n"
		"  -c <location>    	ECEF X,Y,Z in meters (static mode) e.g. 3967283.154,1022538.181,4872414.484\n"
		"  -l <location>    	Lat, lon, height (static mode) e.g. 35.681298,139.766247,10.0\n"
		"  -L <wnslf,dn,dtslf> 	User leap future event in GPS week number, day number, next leap second e.g. 2347,3,19\n"
		"  -t <date,time>   	Scenario start time YYYY/MM/DD,hh:mm:ss\n"
		"  -T <date,time>   	Overwrite TOC and TOE to scenario start time\n"
		"  -d <duration>    	Duration [sec] (dynamic mode max: %.0f, static mode max: %d)\n"
		"  -o <output>      	I/Q sampling data file (default: gpssim.bin)\n"
		"  -s <frequency>   	Sampling frequency [Hz] (default: 2600000)\n"
		"  -b <iq_bits>     	I/Q data format [1/8/16] (default: 16)\n"
		"  -i               	Disable ionospheric delay for spacecraft scenario\n"
		"  -p [fixed_gain]  	Disable path loss and hold power level constant\n"
		"  -v               	Show details about simulated channels\n"
		"  -P [PLoss Mode]  	0: Disable Mode, 1: Auto Mode, 2: Fixed Mode, 3: Variable Mode"
		"  -C [Chan ID]     	Write 16 SV IDs to Simulate in the corresponding channels, set 0 for unselected channels"
		"  -D [Chan Gain]   	Write 16 Channel Gain Values for corresponding channels, value must be within 20 - 50 dB"
		"  -E [Chan Mode]   	0: Turns the corresponding channel OFF, 1: turns it ON"
		"  -F [Fix Gain]    	The Fix Gain value is used in Path Loss Mode: 2"
		"  -G [Gaussian Gain]	Enter Gaussian Gain dB, Range: 0 to 200"
		"  -O [GPS Outage]		Enter Outage Start Time and Stop Time, Example: -O 10,20 where 10 and 20 are in seconds",
		((double)USER_MOTION_SIZE) / 10.0, STATIC_MAX_DURATION);

	return;
}

int main(int argc, char *argv[])
{//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	clock_t tstart,tend;

	FILE *fp;
	
	//extern double gain_dB;
	double gain_dB = 1;
	int outage = 0;
	int gaussian = 0;

	int sv;
	int neph,ieph;
	ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
	gpstime_t g0;

	double llh[3];

	int i;
	channel_t chan[MAX_CHAN];
	double elvmask = 0.0; // in degree

	int ip,qp;
	int iTable;
	short *iq_buff = NULL;
	signed char *iq8_buff = NULL;

	gpstime_t grx;
	double delt;
	int isamp;

	int iumd;
	int numd;
	char umfile[MAX_CHAR];


	int staticLocationMode = FALSE;
	int nmeaGGA = FALSE;
	int umLLH = FALSE;

	char navfile[MAX_CHAR];
	char outfile[MAX_CHAR];

	double samp_freq;
	int iq_buff_size;
	int data_format;

	int result;

	int gain[MAX_CHAN];
	double path_loss;
	double ant_gain;
	int fixed_gain = 128;
	double ant_pat[37];
	int ibs; // boresight angle index

	datetime_t t0,tmin,tmax;
	gpstime_t gmin,gmax;
	double dt;
	int igrx;

	double duration;
	int iduration;
	int verb;

	int timeoverwrite = FALSE; // Overwrite the TOC and TOE in the RINEX file

	ionoutc_t ionoutc;
	int path_loss_enable = TRUE;

	////////////////////////////////////////////////////////////
	// Read options
	////////////////////////////////////////////////////////////

	// Default options
	navfile[0] = 0;
	umfile[0] = 0;
	strcpy(outfile, "gpssim.bin");
	samp_freq = 2.6e6; //Default Time sample is 2600000 Hz (Same set in hack-rf)
	data_format = SC16;
	g0.week = -1; // Invalid start time
	iduration = USER_MOTION_SIZE;
	duration = (double)iduration/10.0; // Default duration
	verb = FALSE;
	ionoutc.enable = TRUE;
	ionoutc.leapen = FALSE;

	if (argc<3)
	{
	 	usage();
	 	exit(1);
	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	while ((result=getopt(argc,argv,"e:u:x:g:c:l:o:s:b:L:T:t:d:ipv:P:C:D:E:F:I:G:O:"))!=-1)
	{
		switch (result)
		{
		case 'e':
			strcpy(navfile, optarg);
			break;
		case 'u':
			strcpy(umfile, optarg);
			nmeaGGA = FALSE;
			umLLH = FALSE;
			break;
		case 'x':
			// Added by romalvarezllorens@gmail.com
			strcpy(umfile, optarg);
			umLLH = TRUE;
			break;
		case 'g':
			strcpy(umfile, optarg);
			nmeaGGA = TRUE;
			break;
		case 'c':
			// Static ECEF coordinates input mode
			staticLocationMode = TRUE;
			sscanf(optarg,"%lf,%lf,%lf",&xyz[0][0],&xyz[0][1],&xyz[0][2]);
			break;
		case 'l':
			// Static geodetic coordinates input mode
			// Added by scateu@gmail.com
			staticLocationMode = TRUE;
			sscanf(optarg,"%lf,%lf,%lf",&llh[0],&llh[1],&llh[2]);
			llh[0] = llh[0] / R2D; // convert to RAD
			llh[1] = llh[1] / R2D; // convert to RAD
			llh2xyz(llh,xyz[0]); // Convert llh to xyz
			break;
		case 'o':
			strcpy(outfile, optarg);
			break;
		case 's':
			samp_freq = atof(optarg);
			if (samp_freq<1.0e6)
			{
				fprintf(stderr, "ERROR: Invalid sampling frequency.\n");
				exit(1);
			}
			break;
		case 'b':
			data_format = atoi(optarg);
			if (data_format!=SC01 && data_format!=SC08 && data_format!=SC16)
			{
				fprintf(stderr, "ERROR: Invalid I/Q data format.\n");
				exit(1);
			}
			break;
		case 'L':
			// enable custom Leap Event
			ionoutc.leapen = TRUE;
			sscanf(optarg,"%d,%d,%d", &ionoutc.wnlsf, &ionoutc.dn, &ionoutc.dtlsf);
			if (ionoutc.dn<1 && ionoutc.dn>7)
			{
				fprintf(stderr, "ERROR: Invalid GPS day number");
				exit(1);
			}
			if (ionoutc.wnlsf<0)
			{
				fprintf(stderr, "ERROR: Invalid GPS week number");
				exit(1);
			}
			if (ionoutc.dtlsf<-128 && ionoutc.dtlsf>127)
			{
				fprintf(stderr, "ERROR: Invalid delta leap second");
				exit(1);
			}
			break;
		case 'T':
			timeoverwrite = TRUE;
			if (strncmp(optarg, "now", 3)==0)
			{
				time_t timer;
				struct tm *gmt;

				time(&timer);
				gmt = gmtime(&timer);

				t0.y = gmt->tm_year+1900;
				t0.m = gmt->tm_mon+1;
				t0.d = gmt->tm_mday;
				t0.hh = gmt->tm_hour;
				t0.mm = gmt->tm_min;
				t0.sec = (double)gmt->tm_sec;

				date2gps(&t0, &g0);

				break;
			}
		case 't':
			sscanf(optarg, "%d/%d/%d,%d:%d:%lf", &t0.y, &t0.m, &t0.d, &t0.hh, &t0.mm, &t0.sec);
			if (t0.y<=1980 || t0.m<1 || t0.m>12 || t0.d<1 || t0.d>31 ||
				t0.hh<0 || t0.hh>23 || t0.mm<0 || t0.mm>59 || t0.sec<0.0 || t0.sec>=60.0)
			{
				fprintf(stderr, "ERROR: Invalid date and time.\n");
				exit(1);
			}
			t0.sec = floor(t0.sec);
			date2gps(&t0, &g0);
			break;
		case 'd':
			duration = atof(optarg);
			break;
		case 'i':
			ionoutc.enable = FALSE; // Disable ionospheric correction
			break;
		case 'p':
			if (optind < argc && argv[optind][0] != '-') // Check if next item is an argument
			{
				fixed_gain = atoi(argv[optind]);
				if (fixed_gain < 1 || fixed_gain > 128)
				{
					fprintf(stderr, "ERROR: Fixed gain must be between 1 and 128.\n");
					exit(1);
				}
				optind++;  // Move past this argument for next iteration
			}
			path_loss_enable = FALSE; // Disable path loss
			break;
		case 'v':
			verb = TRUE;
			break;
		case ':':
		case '?':
			usage();
			exit(1);
		case 'P': //ADDED BY ME		[Notes the Path Loss Mode]
			sscanf(optarg, "%c", pathLossModeChar);
			pathLossMode = atoi(pathLossModeChar);
			if (pathLossMode < 0 || pathLossMode > 3)
			{
				fprintf(stderr, "ERROR: Path Loss Mode must be between 0 and 3\n");
				exit(1);
			}
			fprintf(stderr, "Path Loss Mode: %d\n", pathLossMode);
			break;
		case 'C': //ADDED BY ME		[Converts the input String for Channel ID to Integer Array]
			//Scans the input string 
			sscanf(optarg, "%99s", chanID);
			
			i = 0;
				
			char *token = strtok(chanID, ",");
			while (token != NULL && i < 16)
			{
				chanIDAr[i++] = atoi(token);
				token = strtok(NULL, ",");
			}
			fprintf(stderr, "CHANNEL IDs: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				chanIDAr[0], chanIDAr[1], chanIDAr[2], chanIDAr[3], chanIDAr[4], chanIDAr[5], chanIDAr[6],
				chanIDAr[7], chanIDAr[8], chanIDAr[9], chanIDAr[10], chanIDAr[11], chanIDAr[12], chanIDAr[13],
				chanIDAr[14], chanIDAr[15]);
			break;
		case 'D': //ADDED BY ME		[Converts the input String for Channel Strength to Float Array]
			sscanf(optarg, "%99s", chanStr);
			
			i = 0;
			
			char *token2 = strtok(chanStr, ",");
			while (token2 != NULL && i < 16)
			{
				chanStrAr[i++] = atof(token2);
				token2 = strtok(NULL, ",");
			}
			fprintf(stderr, "CHANNEL Strengths: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
				chanStrAr[0], chanStrAr[1], chanStrAr[2], chanStrAr[3], chanStrAr[4], chanStrAr[5], chanStrAr[6],
				chanStrAr[7], chanStrAr[8], chanStrAr[9], chanStrAr[10], chanStrAr[11], chanStrAr[12], chanStrAr[13],
				chanStrAr[14], chanStrAr[15]);
			break;
		case 'E': //ADDED BY ME		[Converts the input string for Channel Mode to Integer Array]
			sscanf(optarg, "%99s", chanMode);
			
			i = 0;
			
			char *token3 = strtok(chanMode, ",");
			while (token3 != NULL && i < 16)
			{
				chanModeAr[i++] = atoi(token3);
				token3 = strtok(NULL, ",");
			}
			fprintf(stderr, "CHANNEL Modes: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				chanModeAr[0], chanModeAr[1], chanModeAr[2], chanModeAr[3], chanModeAr[4], chanModeAr[5], chanModeAr[6],
				chanModeAr[7], chanModeAr[8], chanModeAr[9], chanModeAr[10], chanModeAr[11], chanModeAr[12], chanModeAr[13],
				chanModeAr[14], chanModeAr[15]);
			break;
		case 'F':
			sscanf(optarg, "%99s", fixStrChar);
			
			i = 0;
			
			fixStr = atof(fixStrChar);
			fprintf(stderr, "Fixed SV Gain: %.4f\n", fixStr);
			break;
		case 'I':
			sscanf(optarg, "%99s", ionoChangeChar);
			
			i = 0;
			
			ionoChange = atof(ionoChangeChar);
			fprintf(stderr, "Ionospheric Effect %: %.4f\n", ionoChange);
			break;
		case 'G': //Handles Gaussian Gain_DB
			gaussian = 1;
			gain_dB = atof(optarg);
			break;
		case 'O': //Handles GPS Outage Times
			outage = 1;
			sscanf(optarg, "%99s", outage_times);
			i = 0;
			char *token4 = strtok(outage_times, ",");
			while(token4 != NULL && i < 2)
			{
				outage_times_Ar[i++] = atoi(token4) * 10;
				token4 = strtok(NULL, ",");
			}
			fprintf(stderr, "Outage Times: %d %d", outage_times_Ar[0], outage_times_Ar[1]);
			break;
		default:
			break;
		}
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (navfile[0]==0)
	{
		fprintf(stderr, "ERROR: GPS ephemeris file is not specified.\n");
		exit(1);
	}

	if (umfile[0]==0 && !staticLocationMode)
	{
		// Default static location; Tokyo
		staticLocationMode = TRUE;
		llh[0] = 35.681298 / R2D;
		llh[1] = 139.766247 / R2D;
		llh[2] = 10.0;
	}

	if (duration<0.0 || (duration>((double)USER_MOTION_SIZE)/10.0 && !staticLocationMode) || (duration>STATIC_MAX_DURATION && staticLocationMode))
	{
		fprintf(stderr, "ERROR: Invalid duration.\n");
		exit(1);
	}
	iduration = (int)(duration*10.0 + 0.5);

	// Buffer size
	samp_freq = floor(samp_freq/10.0);
	iq_buff_size = (int)samp_freq; // samples per 0.1sec
	samp_freq *= 10.0;

	delt = 1.0/samp_freq;

	////////////////////////////////////////////////////////////
	// Receiver position
	////////////////////////////////////////////////////////////

	if (!staticLocationMode)
	{
		// Read user motion file
		if (nmeaGGA==TRUE)
			numd = readNmeaGGA(xyz, umfile);
		else if (umLLH == TRUE)
			numd = readUserMotionLLH(xyz, umfile);
		else
			numd = readUserMotion(xyz, umfile);

		if (numd==-1)
		{
			fprintf(stderr, "ERROR: Failed to open user motion / NMEA GGA file.\n");
			exit(1);
		}
		else if (numd==0)
		{
			fprintf(stderr, "ERROR: Failed to read user motion / NMEA GGA data.\n");
			exit(1);
		}

		// Set simulation duration
		if (numd>iduration)
			numd = iduration;

		// Set user initial position
		xyz2llh(xyz[0], llh);
	}
	else
	{
		// Static geodetic coordinates input mode: "-l"
		// Added by scateu@gmail.com
		fprintf(stderr, "Using static location mode.\n");

		// Set simulation duration
		numd = iduration;

		// Set user initial position
		llh2xyz(llh, xyz[0]);
	}

	fprintf(stderr, "xyz = %11.1f, %11.1f, %11.1f\n", xyz[0][0], xyz[0][1], xyz[0][2]);
	fprintf(stderr, "llh = %11.6f, %11.6f, %11.1f\n", llh[0]*R2D, llh[1]*R2D, llh[2]);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	// Read ephemeris
	////////////////////////////////////////////////////////////

	neph = readRinexNavAll(eph, &ionoutc, navfile); //neph is the variable which has the ephemeris and ionospheric data stored
	                                                //(or rather you can call that data via neph)

	if (neph==0)
	{
		fprintf(stderr, "ERROR: No ephemeris available.\n");
		exit(1);
	}
	else if (neph==-1)
	{
		fprintf(stderr, "ERROR: ephemeris file not found.\n");
		exit(1);	
	}

	if ((verb==TRUE)&&(ionoutc.vflg==TRUE))
	{
		fprintf(stderr, "  %12.3e %12.3e %12.3e %12.3e\n",
			ionoutc.alpha0, ionoutc.alpha1, ionoutc.alpha2, ionoutc.alpha3);
		fprintf(stderr, "  %12.3e %12.3e %12.3e %12.3e\n",
			ionoutc.beta0, ionoutc.beta1, ionoutc.beta2, ionoutc.beta3);
		fprintf(stderr, "   %19.11e %19.11e  %9d %9d\n",
			ionoutc.A0, ionoutc.A1, ionoutc.tot, ionoutc.wnt);
		fprintf(stderr, "%6d\n", ionoutc.dtls);
	}

	for (sv=0; sv<MAX_SAT; sv++)
	{
		if (eph[0][sv].vflg==1)
		{
			gmin = eph[0][sv].toc;
			tmin = eph[0][sv].t;
			break;
		}
	}

	gmax.sec = 0;
	gmax.week = 0;
	tmax.sec = 0;
	tmax.mm = 0;
	tmax.hh = 0;
	tmax.d = 0;
	tmax.m = 0;
	tmax.y = 0;
	for (sv=0; sv<MAX_SAT; sv++)
	{
		if (eph[neph-1][sv].vflg == 1)
		{
			gmax = eph[neph-1][sv].toc;
			tmax = eph[neph-1][sv].t;
			break;
		}
	}

	if (g0.week>=0) // Scenario start time has been set.
	{
		if (timeoverwrite==TRUE)
		{
			gpstime_t gtmp;
			datetime_t ttmp;
			double dsec;

			gtmp.week = g0.week;
			gtmp.sec = (double)(((int)(g0.sec))/7200)*7200.0;

			dsec = subGpsTime(gtmp,gmin); //Calculates difference in seconds between gtmp and gmin

			// Overwrite the UTC reference week number
			ionoutc.wnt = gtmp.week;
			ionoutc.tot = (int)gtmp.sec;

			// Iono/UTC parameters may no longer valid
			//ionoutc.vflg = FALSE;

			// Overwrite the TOC and TOE to the scenario start time
			for (sv=0; sv<MAX_SAT; sv++)
			{
				for (i=0; i<neph; i++)
				{
					if (eph[i][sv].vflg == 1)
					{
						gtmp = incGpsTime(eph[i][sv].toc, dsec); //whatever the difference was between gtmp (g0) and gmin, it is appedned in gtmp
						gps2date(&gtmp,&ttmp);
						eph[i][sv].toc = gtmp;
						eph[i][sv].t = ttmp;

						gtmp = incGpsTime(eph[i][sv].toe, dsec);
						eph[i][sv].toe = gtmp;
					}
				}
			}
		}
		else
		{
			if (subGpsTime(g0, gmin)<0.0 || subGpsTime(gmax, g0)<0.0)
			{
				fprintf(stderr, "ERROR: Invalid start time.\n");
				fprintf(stderr, "tmin = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
					tmin.y, tmin.m, tmin.d, tmin.hh, tmin.mm, tmin.sec,
					gmin.week, gmin.sec);
				fprintf(stderr, "tmax = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
					tmax.y, tmax.m, tmax.d, tmax.hh, tmax.mm, tmax.sec,
					gmax.week, gmax.sec);
				exit(1);
			}
		}
	}
	else
	{
		g0 = gmin;
		t0 = tmin;
	}

	fprintf(stderr, "Start time = %4d/%02d/%02d,%02d:%02d:%02.0f (%d:%.0f)\n",
		t0.y, t0.m, t0.d, t0.hh, t0.mm, t0.sec, g0.week, g0.sec);
	fprintf(stderr, "Duration = %.1f [sec]\n", ((double)numd)/10.0);

	// Select the current set of ephemerides
	ieph = -1;

	for (i=0; i<neph; i++)
	{
		for (sv=0; sv<MAX_SAT; sv++)
		{
			if (eph[i][sv].vflg == 1)
			{
				dt = subGpsTime(g0, eph[i][sv].toc);
				if (dt>=-SECONDS_IN_HOUR && dt<SECONDS_IN_HOUR)
				{
					ieph = i;
					break;
				}
			}
		}

		if (ieph>=0) // ieph has been set
			break;
	}

	if (ieph == -1)
	{
		fprintf(stderr, "ERROR: No current set of ephemerides has been found.\n");
		exit(1);
	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	// Baseband signal buffer and output file
	////////////////////////////////////////////////////////////

	// Allocate I/Q buffer
	//calloc reserves space in memory for IQ and initializes all of it to zero as well
	iq_buff = calloc(2*iq_buff_size, 2);

	if (iq_buff==NULL)
	{
		fprintf(stderr, "ERROR: Failed to allocate 16-bit I/Q buffer.\n");
		exit(1);
	}

	if (data_format==SC08)
	{
		iq8_buff = calloc(2*iq_buff_size, 1);
		if (iq8_buff==NULL)
		{
			fprintf(stderr, "ERROR: Failed to allocate 8-bit I/Q buffer.\n");
			exit(1);
		}
	}
	else if (data_format==SC01)
	{
		iq8_buff = calloc(iq_buff_size/4, 1); // byte = {I0, Q0, I1, Q1, I2, Q2, I3, Q3}
		if (iq8_buff==NULL)
		{
			fprintf(stderr, "ERROR: Failed to allocate compressed 1-bit I/Q buffer.\n");
			exit(1);
		}
	}

	// Open output file
	// "-" can be used as name for stdout
	if(strcmp("-", outfile)){
		if (NULL==(fp=fopen(outfile,"wb")))
		{
			fprintf(stderr, "ERROR: Failed to open output file.\n");
			exit(1);
		}
	}else{
		fp = stdout;
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	// Initialize channels
	////////////////////////////////////////////////////////////

	// Clear all channels
	for (i=0; i<MAX_CHAN; i++)
		chan[i].prn = 0;

	// Clear satellite allocation flag
	for (sv=0; sv<MAX_SAT; sv++)
		allocatedSat[sv] = -1;

	// Initial reception time
	grx = incGpsTime(g0, 0.0);

	// Allocate visible satellites
	allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[0], elvmask);

	for(i=0; i<MAX_CHAN; i++)
	{
		if (chan[i].prn>0)
			fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,
				chan[i].azel[0]*R2D, chan[i].azel[1]*R2D, chan[i].rho0.d, chan[i].rho0.iono_delay);
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	// Receiver antenna gain pattern
	////////////////////////////////////////////////////////////

	for (i=0; i<37; i++)
		ant_pat[i] = pow(10.0, -ant_pat_db[i]/20.0);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	// Generate baseband signals
	////////////////////////////////////////////////////////////

	tstart = clock();

	// Update receiver time
	grx = incGpsTime(grx, 0.1);

	for (iumd=1; iumd<numd; iumd++)
	{
		if (outage == 1)
		{
			if (iumd > outage_times_Ar[0] && iumd < outage_times_Ar[1])
			{
				for (isamp=0; isamp<iq_buff_size; isamp++)
				{
					int i_acc = 0;
					int q_acc = 0;
					for (i=0; i<MAX_CHAN; i++)
					{
						if (chan[i].prn>0)
						{
							ip = cosTable512[iTable] * gain[i];
							qp = sinTable512[iTable] * gain[i];
							
							i_acc += ip;
							q_acc += qp;
						}
					}
					i_acc = (i_acc+64)>>7;
					q_acc = (q_acc+64)>>7;
					
					iq_buff[isamp*2] = (short)i_acc;
					iq_buff[isamp*2+1] = (short)q_acc;
				}
				if (data_format==SC01)
				{
					for (isamp=0; isamp<2*iq_buff_size; isamp++)
					{
						if (isamp%8==0)
							iq8_buff[isamp/8] = 0x00;
		
						iq8_buff[isamp/8] |= (iq_buff[isamp]>0?0x01:0x00)<<(7-isamp%8);
					}
		
					fwrite(iq8_buff, 1, iq_buff_size/4, fp);
				}
				else if (data_format==SC08)
				{
					for (isamp=0; isamp<2*iq_buff_size; isamp++)
						iq8_buff[isamp] = iq_buff[isamp]>>4; // 12-bit bladeRF -> 8-bit HackRF
					fwrite(iq8_buff, 1, 2*iq_buff_size, fp);
				}
				else // data_format==SC16
				{
					fwrite(iq_buff, 2, 2*iq_buff_size, fp);
				}
				
				//
				// Update navigation message and channel allocation every 30 seconds
				//
				igrx = (int)(grx.sec*10.0+0.5);
		
				if (igrx%300==0) // Every 30 seconds
				{
					// Update navigation message
					for (i=0; i<MAX_CHAN; i++)
					{
						if (chan[i].prn>0)
							generateNavMsg(grx, &chan[i], 0);
					}
		
					// Refresh ephemeris and subframes
					// Quick and dirty fix. Need more elegant way.
					for (sv=0; sv<MAX_SAT; sv++)
					{
						if (eph[ieph+1][sv].vflg==1)
						{
							dt = subGpsTime(eph[ieph+1][sv].toc, grx);
							if (dt<SECONDS_IN_HOUR)
							{
								ieph++;
		
								for (i=0; i<MAX_CHAN; i++)
								{
									// Generate new subframes if allocated
									if (chan[i].prn!=0)
										eph2sbf(eph[ieph][chan[i].prn-1], ionoutc, chan[i].sbf);
								}
							}
		
							break;
						}
					}
		
					// Update channel allocation
					if (!staticLocationMode)
						allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[iumd], elvmask);
					else
						allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[0], elvmask);
		
					// Show details about simulated channels
					if (verb==TRUE)
					{
						fprintf(stderr, "\n");
						for (i=0; i<MAX_CHAN; i++)
						{
							if (chan[i].prn>0)
								fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,
									chan[i].azel[0]*R2D, chan[i].azel[1]*R2D, chan[i].rho0.d, chan[i].rho0.iono_delay);
						}
					}
				}
		
				// Update receiver time
				grx = incGpsTime(grx, 0.1);
		
				// Update time counter
				fprintf(stderr, "\rTime into ORORORO = %4.1f", subGpsTime(grx, g0));
				fflush(stdout);
			}
			else
			{
				for (i=0; i<MAX_CHAN; i++)
				{
					if (chan[i].prn>0)
					{
						// Refresh code phase and data bit counters
						range_t rho;
						sv = chan[i].prn-1;
		
						// Current pseudorange
						if (!staticLocationMode)
							computeRange(&rho, eph[ieph][sv], &ionoutc, grx, xyz[iumd]);
						else
							computeRange(&rho, eph[ieph][sv], &ionoutc, grx, xyz[0]);
		
						chan[i].azel[0] = rho.azel[0];
						chan[i].azel[1] = rho.azel[1];
		
						// Update code phase and data bit counters
						computeCodePhase(&chan[i], rho, 0.1);
#ifndef FLOAT_CARR_PHASE
						chan[i].carr_phasestep = (int)round(512.0 * 65536.0 * chan[i].f_carr * delt);
#endif
						// Path loss
						path_loss = 20200000.0/rho.d;
		
						// Receiver antenna gain
						ibs = (int)((90.0-rho.azel[1]*R2D)/5.0); // covert elevation to boresight
						ant_gain = ant_pat[ibs];
		
						// Signal gain
						if (path_loss_enable == TRUE)
						{
							switch(pathLossMode)
							{
							case 0: //ADDED BY ME		[DISBALE MODE:	Allocates Satellites in for loop, All Channels are set at Max Gain Setting]
								gain[0] = 1*128.0*ant_gain;
								gain[1] = 1*128.0*ant_gain;
								gain[2] = 1*128.0*ant_gain;
								gain[3] = 1*128.0*ant_gain;
								gain[4] = 1*128.0*ant_gain;
								gain[5] = 1*128.0*ant_gain;
								gain[6] = 1*128.0*ant_gain;
								gain[7] = 1*128.0*ant_gain;
								gain[8] = 1*128.0*ant_gain;
								gain[9] = 1*128.0*ant_gain;
								gain[10] = 1*128.0*ant_gain;
								gain[11] = 1*128.0*ant_gain;
								gain[12] = 1*128.0*ant_gain;
								gain[13] = 1*128.0*ant_gain;
								gain[14] = 1*128.0*ant_gain;
								gain[15] = 1*128.0*ant_gain;
								break;
							case 1: //ADDED BY ME		[AUTO MODE:	Allocates Satellites in for loop, Channel Gain is set Automatically]
								gain[i] = (int)(path_loss * ant_gain * 128.0);
								break;
							case 2: //ADDED BY ME		[FIX MODE:	Allocates Satellites Defined by User Using chanIDAr array, Gain is set at a user-defined fixed value]
								//gain_factor = 0.0079 * exp(0.0892 * fixStr);
								gain_factor = 0.0015 * exp(0.1163 * fixStr);
								for (int i = 0; i < 16; i++)
								{
									if (chanModeAr[i] == 1 && indexPLM2 == -1)
									{
										indexPLM2 = i;
									}
								}
								for (int j = 0; j < 16; j++)
								{
									if (indexPLM2 == j && chanModeAr[j] == 1)
									{
										gain[indexPLM2] = 1*128.0*ant_gain;
									}
									else if (chanModeAr[j] == 1)
									{
										gain[j] = gain_factor*128.0*ant_gain;
									}
								}
								
								/*
								gain[1] = 1*128.0*ant_gain * chanModeAr[1];
								gain[2] = gain_factor*128.0*ant_gain * chanModeAr[2];
								gain[3] = gain_factor*128.0*ant_gain * chanModeAr[3];
								gain[4] = gain_factor*128.0*ant_gain * chanModeAr[4];
								gain[5] = gain_factor*128.0*ant_gain * chanModeAr[5];
								gain[6] = gain_factor*128.0*ant_gain * chanModeAr[6];
								gain[7] = gain_factor*128.0*ant_gain * chanModeAr[7];
								gain[8] = gain_factor*128.0*ant_gain * chanModeAr[8];
								gain[9] = gain_factor*128.0*ant_gain * chanModeAr[9];
								gain[10] = gain_factor*128.0*ant_gain * chanModeAr[10];
								gain[11] = gain_factor*128.0*ant_gain * chanModeAr[11];
								gain[12] = gain_factor*128.0*ant_gain * chanModeAr[12];
								gain[13] = gain_factor*128.0*ant_gain * chanModeAr[13];
								gain[14] = gain_factor*128.0*ant_gain * chanModeAr[14];
								gain[15] = gain_factor*128.0*ant_gain;
								*/
								break;
							case 3: //ADDED BY ME		[VAR MODE:	Allocates Satellites Defined by User Using chanIDAr array, Gain is set at user defined values given by chanStrAr]
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[0]) * chanModeAr[0];
								gain[0] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[1]) * chanModeAr[1];
								gain[1] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[2]) * chanModeAr[2];
								gain[2] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[3]) * chanModeAr[3];
								gain[3] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[4]) * chanModeAr[4];
								gain[4] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[5]) * chanModeAr[5];
								gain[5] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[6]) * chanModeAr[6];
								gain[6] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[7]) * chanModeAr[7];
								gain[7] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[8]) * chanModeAr[8];
								gain[8] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[9]) * chanModeAr[9];
								gain[9] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[10]) * chanModeAr[10];
								gain[10] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[11]) * chanModeAr[11];
								gain[11] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[12]) * chanModeAr[12];
								gain[12] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[13]) * chanModeAr[13];
								gain[13] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[14]) * chanModeAr[14];
								gain[14] = gain_factor * 128.0 * ant_gain;
								gain_factor = 0.0015 * exp(0.1163 * chanStrAr[15]) * chanModeAr[15];
								gain[15] = gain_factor * 128.0 * ant_gain;
								break;
							default:
								break;
							}
		
						}
						else
							gain[i] = fixed_gain; // hold the power level constant
					}
				}
		
				for (isamp=0; isamp<iq_buff_size; isamp++)
				{
					int i_acc = 0;
					int q_acc = 0;
		
					for (i=0; i<MAX_CHAN; i++)
					{
						if (chan[i].prn>0)
						{
#ifdef FLOAT_CARR_PHASE
							iTable = (int)floor(chan[i].carr_phase*512.0); //rounds to nearest whole number
#else
							iTable = (chan[i].carr_phase >> 16) & 0x1ff; // 9-bit index //Shifts bits of chan[i].carr_phaser 16 places to the right, ANDs with 0x1ff
#endif
							if (gaussian == 1)
							{
								ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i] + generate_gaussian_noise(0, 1, gain_dB); //data x C/A code x cos(phase) x gain
								qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i] + generate_gaussian_noise(0, 1, gain_dB); //data x C/A code x sin(phase) x gain
							}
							else
							{
								ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i];
								qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];
							}
		
							// Accumulate for all visible satellites
							i_acc += ip;
							q_acc += qp;
		
							// Update code phase
							chan[i].code_phase += chan[i].f_code * delt;
		
							if (chan[i].code_phase>=CA_SEQ_LEN)
							{
								chan[i].code_phase -= CA_SEQ_LEN;
		
								chan[i].icode++;
		
								if (chan[i].icode>=20) // 20 C/A codes = 1 navigation data bit
								{
									chan[i].icode = 0;
									chan[i].ibit++;
		
									if (chan[i].ibit>=30) // 30 navigation data bits = 1 word
									{
										chan[i].ibit = 0;
										chan[i].iword++;
										/*
										if (chan[i].iword>=N_DWRD)
											fprintf(stderr, "\nWARNING: Subframe word buffer overflow.\n");
										*/
									}
		
									// Set new navigation data bit
									chan[i].dataBit = (int)((chan[i].dwrd[chan[i].iword]>>(29-chan[i].ibit)) & 0x1UL)*2-1;
								}
							}
		
							// Set current code chip
							chan[i].codeCA = chan[i].ca[(int)chan[i].code_phase]*2-1;
		
							// Update carrier phase
#ifdef FLOAT_CARR_PHASE
							chan[i].carr_phase += chan[i].f_carr * delt;
		
							if (chan[i].carr_phase >= 1.0)
								chan[i].carr_phase -= 1.0;
							else if (chan[i].carr_phase<0.0)
								chan[i].carr_phase += 1.0;
#else
							chan[i].carr_phase += chan[i].carr_phasestep;
#endif
						}
					}
		
					// Scaled by 2^7
					i_acc = (i_acc+64)>>7;
					q_acc = (q_acc+64)>>7;
		
					// Store I/Q samples into buffer
					iq_buff[isamp*2] = (short)i_acc;
					iq_buff[isamp*2+1] = (short)q_acc;
				}
		
				if (data_format==SC01)
				{
					for (isamp=0; isamp<2*iq_buff_size; isamp++)
					{
						if (isamp%8==0)
							iq8_buff[isamp/8] = 0x00;
		
						iq8_buff[isamp/8] |= (iq_buff[isamp]>0?0x01:0x00)<<(7-isamp%8);
					}
		
					fwrite(iq8_buff, 1, iq_buff_size/4, fp);
				}
				else if (data_format==SC08)
				{
					for (isamp=0; isamp<2*iq_buff_size; isamp++)
						iq8_buff[isamp] = iq_buff[isamp]>>4; // 12-bit bladeRF -> 8-bit HackRF
		
					fwrite(iq8_buff, 1, 2*iq_buff_size, fp);
				}
				else // data_format==SC16
				{
					fwrite(iq_buff, 2, 2*iq_buff_size, fp);
				}
		
				//
				// Update navigation message and channel allocation every 30 seconds
				//
		
				igrx = (int)(grx.sec*10.0+0.5);
		
				if (igrx%300==0) // Every 30 seconds
				{
					// Update navigation message
					for (i=0; i<MAX_CHAN; i++)
					{
						if (chan[i].prn>0)
							generateNavMsg(grx, &chan[i], 0);
					}
		
					// Refresh ephemeris and subframes
					// Quick and dirty fix. Need more elegant way.
					for (sv=0; sv<MAX_SAT; sv++)
					{
						if (eph[ieph+1][sv].vflg==1)
						{
							dt = subGpsTime(eph[ieph+1][sv].toc, grx);
							if (dt<SECONDS_IN_HOUR)
							{
								ieph++;
		
								for (i=0; i<MAX_CHAN; i++)
								{
									// Generate new subframes if allocated
									if (chan[i].prn!=0)
										eph2sbf(eph[ieph][chan[i].prn-1], ionoutc, chan[i].sbf);
								}
							}
		
							break;
						}
					}
		
					// Update channel allocation
					if (!staticLocationMode)
						allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[iumd], elvmask);
					else
						allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[0], elvmask);
		
					// Show details about simulated channels
					if (verb==TRUE)
					{
						fprintf(stderr, "\n");
						for (i=0; i<MAX_CHAN; i++)
						{
							if (chan[i].prn>0)
								fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,
									chan[i].azel[0]*R2D, chan[i].azel[1]*R2D, chan[i].rho0.d, chan[i].rho0.iono_delay);
						}
					}
				}
		
				// Update receiver time
				grx = incGpsTime(grx, 0.1);
		
				// Update time counter
				fprintf(stderr, "\rTime into run = %4.1f", subGpsTime(grx, g0));
				fflush(stdout);
			}
		}
		else
		{
			for (i=0; i<MAX_CHAN; i++)
			{
				if (chan[i].prn>0)
				{
					// Refresh code phase and data bit counters
					range_t rho;
					sv = chan[i].prn-1;
	
					// Current pseudorange
					if (!staticLocationMode)
						computeRange(&rho, eph[ieph][sv], &ionoutc, grx, xyz[iumd]);
					else
						computeRange(&rho, eph[ieph][sv], &ionoutc, grx, xyz[0]);
	
					chan[i].azel[0] = rho.azel[0];
					chan[i].azel[1] = rho.azel[1];
	
					// Update code phase and data bit counters
					computeCodePhase(&chan[i], rho, 0.1);
#ifndef FLOAT_CARR_PHASE
					chan[i].carr_phasestep = (int)round(512.0 * 65536.0 * chan[i].f_carr * delt);
#endif
					// Path loss
					path_loss = 20200000.0/rho.d;
	
					// Receiver antenna gain
					ibs = (int)((90.0-rho.azel[1]*R2D)/5.0); // covert elevation to boresight
					ant_gain = ant_pat[ibs];
	
					// Signal gain
					if (path_loss_enable == TRUE)
					{
						switch(pathLossMode)
						{
						case 0: //ADDED BY ME		[DISBALE MODE:	Allocates Satellites in for loop, All Channels are set at Max Gain Setting]
							gain[0] = 1*128.0*ant_gain;
							gain[1] = 1*128.0*ant_gain;
							gain[2] = 1*128.0*ant_gain;
							gain[3] = 1*128.0*ant_gain;
							gain[4] = 1*128.0*ant_gain;
							gain[5] = 1*128.0*ant_gain;
							gain[6] = 1*128.0*ant_gain;
							gain[7] = 1*128.0*ant_gain;
							gain[8] = 1*128.0*ant_gain;
							gain[9] = 1*128.0*ant_gain;
							gain[10] = 1*128.0*ant_gain;
							gain[11] = 1*128.0*ant_gain;
							gain[12] = 1*128.0*ant_gain;
							gain[13] = 1*128.0*ant_gain;
							gain[14] = 1*128.0*ant_gain;
							gain[15] = 1*128.0*ant_gain;
							break;
						case 1: //ADDED BY ME		[AUTO MODE:	Allocates Satellites in for loop, Channel Gain is set Automatically]
							gain[i] = (int)(path_loss * ant_gain * 128.0);
							break;
						case 2: //ADDED BY ME		[FIX MODE:	Allocates Satellites Defined by User Using chanIDAr array, Gain is set at a user-defined fixed value]
							//gain_factor = 0.0079 * exp(0.0892 * fixStr);
							gain_factor = 0.0015 * exp(0.1163 * fixStr);
							for (int i = 0; i < 16; i++)
							{
								if (chanModeAr[i] == 1 && indexPLM2 == -1)
								{
									indexPLM2 = i;
								}
							}
							for (int j = 0; j < 16; j++)
							{
								if (indexPLM2 == j && chanModeAr[j] == 1)
								{
									gain[indexPLM2] = 1*128.0*ant_gain;
								}
								else if (chanModeAr[j] == 1)
								{
									gain[j] = gain_factor*128.0*ant_gain;
								}
							}
							
							/*
							gain[1] = 1*128.0*ant_gain * chanModeAr[1];
							gain[2] = gain_factor*128.0*ant_gain * chanModeAr[2];
							gain[3] = gain_factor*128.0*ant_gain * chanModeAr[3];
							gain[4] = gain_factor*128.0*ant_gain * chanModeAr[4];
							gain[5] = gain_factor*128.0*ant_gain * chanModeAr[5];
							gain[6] = gain_factor*128.0*ant_gain * chanModeAr[6];
							gain[7] = gain_factor*128.0*ant_gain * chanModeAr[7];
							gain[8] = gain_factor*128.0*ant_gain * chanModeAr[8];
							gain[9] = gain_factor*128.0*ant_gain * chanModeAr[9];
							gain[10] = gain_factor*128.0*ant_gain * chanModeAr[10];
							gain[11] = gain_factor*128.0*ant_gain * chanModeAr[11];
							gain[12] = gain_factor*128.0*ant_gain * chanModeAr[12];
							gain[13] = gain_factor*128.0*ant_gain * chanModeAr[13];
							gain[14] = gain_factor*128.0*ant_gain * chanModeAr[14];
							gain[15] = gain_factor*128.0*ant_gain;
							*/
							break;
						case 3: //ADDED BY ME		[VAR MODE:	Allocates Satellites Defined by User Using chanIDAr array, Gain is set at user defined values given by chanStrAr]
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[0]) * chanModeAr[0];
							gain[0] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[1]) * chanModeAr[1];
							gain[1] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[2]) * chanModeAr[2];
							gain[2] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[3]) * chanModeAr[3];
							gain[3] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[4]) * chanModeAr[4];
							gain[4] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[5]) * chanModeAr[5];
							gain[5] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[6]) * chanModeAr[6];
							gain[6] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[7]) * chanModeAr[7];
							gain[7] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[8]) * chanModeAr[8];
							gain[8] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[9]) * chanModeAr[9];
							gain[9] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[10]) * chanModeAr[10];
							gain[10] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[11]) * chanModeAr[11];
							gain[11] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[12]) * chanModeAr[12];
							gain[12] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[13]) * chanModeAr[13];
							gain[13] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[14]) * chanModeAr[14];
							gain[14] = gain_factor * 128.0 * ant_gain;
							gain_factor = 0.0015 * exp(0.1163 * chanStrAr[15]) * chanModeAr[15];
							gain[15] = gain_factor * 128.0 * ant_gain;
							break;
						default:
							break;
						}
	
					}
					else
						gain[i] = fixed_gain; // hold the power level constant
				}
			}
	
			for (isamp=0; isamp<iq_buff_size; isamp++)
			{
				int i_acc = 0;
				int q_acc = 0;
	
				for (i=0; i<MAX_CHAN; i++)
				{
					if (chan[i].prn>0)
					{
#ifdef FLOAT_CARR_PHASE
						iTable = (int)floor(chan[i].carr_phase*512.0); //rounds to nearest whole number
#else
						iTable = (chan[i].carr_phase >> 16) & 0x1ff; // 9-bit index //Shifts bits of chan[i].carr_phaser 16 places to the right, ANDs with 0x1ff
#endif
						if (gaussian == 1)
						{
							ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i] + generate_gaussian_noise(0, 1, gain_dB); //data x C/A code x cos(phase) x gain
							qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i] + generate_gaussian_noise(0, 1, gain_dB); //data x C/A code x sin(phase) x gain
						}
						else
						{
							ip = chan[i].dataBit * chan[i].codeCA * cosTable512[iTable] * gain[i];
							qp = chan[i].dataBit * chan[i].codeCA * sinTable512[iTable] * gain[i];
						}
	
						// Accumulate for all visible satellites
						i_acc += ip;
						q_acc += qp;
	
						// Update code phase
						chan[i].code_phase += chan[i].f_code * delt;
	
						if (chan[i].code_phase>=CA_SEQ_LEN)
						{
							chan[i].code_phase -= CA_SEQ_LEN;
	
							chan[i].icode++;
	
							if (chan[i].icode>=20) // 20 C/A codes = 1 navigation data bit
							{
								chan[i].icode = 0;
								chan[i].ibit++;
	
								if (chan[i].ibit>=30) // 30 navigation data bits = 1 word
								{
									chan[i].ibit = 0;
									chan[i].iword++;
									/*
									if (chan[i].iword>=N_DWRD)
										fprintf(stderr, "\nWARNING: Subframe word buffer overflow.\n");
									*/
								}
	
								// Set new navigation data bit
								chan[i].dataBit = (int)((chan[i].dwrd[chan[i].iword]>>(29-chan[i].ibit)) & 0x1UL)*2-1;
							}
						}
	
						// Set current code chip
						chan[i].codeCA = chan[i].ca[(int)chan[i].code_phase]*2-1;
	
						// Update carrier phase
#ifdef FLOAT_CARR_PHASE
						chan[i].carr_phase += chan[i].f_carr * delt;
	
						if (chan[i].carr_phase >= 1.0)
							chan[i].carr_phase -= 1.0;
						else if (chan[i].carr_phase<0.0)
							chan[i].carr_phase += 1.0;
#else
						chan[i].carr_phase += chan[i].carr_phasestep;
#endif
					}
				}
	
				// Scaled by 2^7
				i_acc = (i_acc+64)>>7;
				q_acc = (q_acc+64)>>7;
	
				// Store I/Q samples into buffer
				iq_buff[isamp*2] = (short)i_acc;
				iq_buff[isamp*2+1] = (short)q_acc;
			}
	
			if (data_format==SC01)
			{
				for (isamp=0; isamp<2*iq_buff_size; isamp++)
				{
					if (isamp%8==0)
						iq8_buff[isamp/8] = 0x00;
	
					iq8_buff[isamp/8] |= (iq_buff[isamp]>0?0x01:0x00)<<(7-isamp%8);
				}
	
				fwrite(iq8_buff, 1, iq_buff_size/4, fp);
			}
			else if (data_format==SC08)
			{
				for (isamp=0; isamp<2*iq_buff_size; isamp++)
					iq8_buff[isamp] = iq_buff[isamp]>>4; // 12-bit bladeRF -> 8-bit HackRF
	
				fwrite(iq8_buff, 1, 2*iq_buff_size, fp);
			}
			else // data_format==SC16
			{
				fwrite(iq_buff, 2, 2*iq_buff_size, fp);
			}
	
			//
			// Update navigation message and channel allocation every 30 seconds
			//
	
			igrx = (int)(grx.sec*10.0+0.5);
	
			if (igrx%300==0) // Every 30 seconds
			{
				// Update navigation message
				for (i=0; i<MAX_CHAN; i++)
				{
					if (chan[i].prn>0)
						generateNavMsg(grx, &chan[i], 0);
				}
	
				// Refresh ephemeris and subframes
				// Quick and dirty fix. Need more elegant way.
				for (sv=0; sv<MAX_SAT; sv++)
				{
					if (eph[ieph+1][sv].vflg==1)
					{
						dt = subGpsTime(eph[ieph+1][sv].toc, grx);
						if (dt<SECONDS_IN_HOUR)
						{
							ieph++;
	
							for (i=0; i<MAX_CHAN; i++)
							{
								// Generate new subframes if allocated
								if (chan[i].prn!=0)
									eph2sbf(eph[ieph][chan[i].prn-1], ionoutc, chan[i].sbf);
							}
						}
	
						break;
					}
				}
	
				// Update channel allocation
				if (!staticLocationMode)
					allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[iumd], elvmask);
				else
					allocateChannel(chan, eph[ieph], ionoutc, grx, xyz[0], elvmask);
	
				// Show details about simulated channels
				if (verb==TRUE)
				{
					fprintf(stderr, "\n");
					for (i=0; i<MAX_CHAN; i++)
					{
						if (chan[i].prn>0)
							fprintf(stderr, "%02d %6.1f %5.1f %11.1f %5.1f\n", chan[i].prn,
								chan[i].azel[0]*R2D, chan[i].azel[1]*R2D, chan[i].rho0.d, chan[i].rho0.iono_delay);
					}
				}
			}
	
			// Update receiver time
			grx = incGpsTime(grx, 0.1);
	
			// Update time counter
			fprintf(stderr, "\rTime into run = %4.1f", subGpsTime(grx, g0));
			fflush(stdout);
		}
	}

	tend = clock();

	fprintf(stderr, "\nDone!\n");

	// Free I/Q buffer
	free(iq_buff);

	// Close file
	fclose(fp);

	// Process time
	fprintf(stderr, "Process time = %.1f [sec]\n", (double)(tend-tstart)/CLOCKS_PER_SEC);

	return(0);
}
