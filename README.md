# GPS-SIM-MODIFIED
Based on the working of osqzz-gps-sdr-sim, the modifications are made to include gaussian noise, signal outage and various channel impairments

Usage: gps-sdr-sim [options]
Options:

  -e <gps_nav>          RINEX navigation file for GPS ephemerides (required)
  
  -u <user_motion>      User motion file in ECEF x, y, z format (dynamic mode)
  
  -x <user_motion>      User motion file in lat, lon, height format (dynamic mode)
  
  -g <nmea_gga>         NMEA GGA stream (dynamic mode)
  
  -c <location>         ECEF X,Y,Z in meters (static mode) e.g. 3967283.154,1022538.181,4872414.484
  
  -l <location>         Lat, lon, height (static mode) e.g. 35.681298,139.766247,10.0
  
  -L <wnslf,dn,dtslf>   User leap future event in GPS week number, day number, next leap second e.g. 2347,3,19
  
  -t <date,time>        Scenario start time YYYY/MM/DD,hh:mm:ss
  
  -T <date,time>        Overwrite TOC and TOE to scenario start time
  
  -d <duration>         Duration [sec] (dynamic mode max: 900, static mode max: 86400)
  
  -o <output>           I/Q sampling data file (default: gpssim.bin)
  
  -s <frequency>        Sampling frequency [Hz] (default: 2600000)
  
  -b <iq_bits>          I/Q data format [1/8/16] (default: 16)
  
  -i                    Disable ionospheric delay for spacecraft scenario
  
  -p [fixed_gain]       Disable path loss and hold power level constant
  
  -v                    Show details about simulated channels
  
  -P [PLoss Mode]       0: Disable Mode, 1: Auto Mode, 2: Fixed Mode, 3: Variable Mode  
  
  -C [Chan ID]            Write 16 SV IDs to Simulate in the corresponding channels, set 0 for unselected channels  
  
  -D [Chan Gain]        Write 16 Channel Gain Values for corresponding channels, value must be within 20 - 50 dB  
  
  -E [Chan Mode]        0: Turns the corresponding channel OFF, 1: turns it ON  
  
  -F [Fix Gain]           The Fix Gain value is used in Path Loss Mode: 2  
  
  -G [Gaussian Gain]     Enter Gaussian Gain dB, Range: 0 to 200  
  
  -O [GPS Outage]                Enter Outage Start Time and Stop Time, Example: -O 10,20 where 10 and 20 are in seconds
