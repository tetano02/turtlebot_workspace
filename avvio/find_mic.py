import pyaudio
import sys

try:
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    print("Trovati dispositivi di input (microfoni):")
    print("="*40)

    for i in range(0, numdevices):
        device_info = p.get_device_info_by_host_api_device_index(0, i)
        if (device_info.get('maxInputChannels')) > 0:
            print(f"Index {i} - {device_info.get('name')}")

    print("="*40)
    print("Copia il numero di 'Index' del tuo microfono.")
    print("Consiglio: prova prima 'pulse' o 'default' se esistono.")

except Exception as e:
    print(f"Errore: {e}")
    print("Assicurati di aver installato 'pyaudio' (pip install pyaudio)")
finally:
    if 'p' in locals():
        p.terminate()
