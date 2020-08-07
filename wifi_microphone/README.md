# wifi microphone on esp8266

  In project uses an  I2S microphone SPH0645(INMP441), ESP-12E. L/R pin of mic connected to ground.
  To play audio insert in vlc  "http:\\\wifi-mic.local:8080\rec.wav"  or "http:\\\ip_of_mic:8080\rec.wav".
  Initially, you need to connect to the access point "wifi-mic_ap" and configure the access settings.
  You can find the ip address from the UART terminal, or from the program on your smartphone - "bonjour browser".
  Examples of recording audio 16 and 24 bits - tttt_16.wav , tttt_24.wav
  
  serial_audio.exe - program for wifi_microphone,  it allows you to testing  microphone through a serial port (uncomment  "//#define NO_WIFI").
  
 My microphone module  INMP441 broke down after a while, i tested another module - GY-SPH0645, it gives a better sound.
 
 UPD: the microphone INMP441 from the new batch also gives high-quality sound.


# License

  This software is provided under the  <a href="http://unlicense.org/" rel="nofollow">UNLICENSE</a>

