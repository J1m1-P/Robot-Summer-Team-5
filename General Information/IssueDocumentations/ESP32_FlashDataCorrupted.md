# ESP32 \- Flash Data Corrupted Issue

# Error Message: 

A fatal error occurred: MD5 of file does not match data in flash\!

# Symptoms: 

- All the codes that are different from the code that is already uploaded to the board cannot be flashed. That includes differences both in the code and spacing between lines. 

# Possible Cause: 

- Unstable supplied voltage to the ESP32 during flashing.

# Fix: 

1. Open terminal  
2. Run “python \--version” and “pip \--version” to ensure python is downloaded already. If not, download it first.   
3. Install esptool by running “pip install esptool” and ensure it is downloaded by running “pip show esptool” or “python \-m esptool version”.   
4. Connect your device to the faulty ESP32.   
5. Run “mode” in the terminal, find the port the ESP32 is connected to   
   1. There could be more than one port connecting to the ESP32 (eg. both COM3 and COM5 are connected). In that case, choosing either is fine.   
6. Assuming we use COM5, run “python \-m esptool \--chip esp32s3 \--port COM5 flash-id” to check its state.   
7. Run “python \-m esptool \--chip esp32s3 \--port COM5 erase-flash”. This would erase the corrupted flash data on the ESP32.  
   1. This removes:   
      1.  Firmware  
      2. saved settings  
      3. NVS data  
      4. Wi-Fi credentials  
      5. partition table  
      6. corrupted flash contents   
8. Now it should be fixed, try flashing with some code. 