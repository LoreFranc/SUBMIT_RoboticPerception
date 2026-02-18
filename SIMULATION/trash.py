import zmq
import json
import re
import sys
import time

# --- CONFIGURAZIONE ---
MADS_ENDPOINT = "tcp://raspberrypi.local:9091"
TOPIC_FILTER = ["odometry_filter"]        

def clean_string_readable(s):
    """
    Rimuove i byte binari e lascia solo testo ASCII stampabile
    per permetterti di leggere il contenuto 'sporco'.
    """
    # Tiene solo caratteri ASCII stampabili (32-126)
    return re.sub(r'[^\x20-\x7E]', '.', s)

def scan_all_numbers(text):
    """
    Trova TUTTE le coppie "chiave": numero nel testo.
    Utile per capire quali dati sono sopravvissuti.
    """
    # Cerca pattern generico: "parola" : numero
    # Accetta anche chiavi corrotte (es. "0stamp")
    pattern = r'"([a-zA-Z0-9_]+)"\s*[:=]\s*([-+0-9.eE]+)'
    matches = re.findall(pattern, text)
    return matches

def scan_vectors(text):
    """
    Trova i vettori [x, y]
    """
    pattern = r'"([a-zA-Z0-9_]+)"\s*[:=]\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)'
    matches = re.findall(pattern, text)
    return matches

def main():
    print(f"--- AVVIO SUPER INSPECTOR ---")
    print(f"Connecting to: {MADS_ENDPOINT}")
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    try:
        socket.connect(MADS_ENDPOINT)
    except Exception as e:
        print(f"Connection Failed: {e}")
        sys.exit(1)
    
    for topic in TOPIC_FILTER:
        socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        print(f"Subscribed: {topic}")

    print("\nIn attesa di pacchetti... (Premi Ctrl+C per fermare)\n")

    msg_count = 0

    while True:
        try:
            # 1. Ricezione
            msg = socket.recv_multipart()
            payload = msg[1]
            
            try: topic = msg[0].decode('utf-8')
            except: topic = "BINARY"

            if topic not in TOPIC_FILTER: continue
            
            msg_count += 1
            print("="*80)
            print(f"MSG #{msg_count} | TOPIC: {topic} | SIZE: {len(payload)} bytes")

            # 2. Decodifica "Sporca"
            # Cerchiamo la prima graffa '{' per ignorare l'header
            start_idx = payload.find(b'{')
            if start_idx == -1:
                print("   [ERROR] Nessun JSON start '{' trovato.")
                continue

            # Convertiamo in stringa sostituendo gli errori
            raw_text = payload[start_idx:].decode('utf-8', errors='replace')
            
            # Pulisci per visualizzazione
            readable_text = clean_string_readable(raw_text)
            
            print(f"\n--- 1. CONTENUTO GREZZO (Primi 200 char) ---")
            print(f"{readable_text[:200]} ...")

            # 3. MAGIC SCANNER (Cosa vede il computer?)
            print(f"\n--- 2. VARIABILI TROVATE (Magic Scanner) ---")
            
            # Numeri singoli
            pairs = scan_all_numbers(raw_text)
            found_keys = []
            if pairs:
                for key, val in pairs:
                    print(f"   > '{key}': {val}")
                    found_keys.append(key)
            else:
                print("   [NESSUN NUMERO TROVATO]")

            # Vettori
            vectors = scan_vectors(raw_text)
            if vectors:
                for key, v1, v2 in vectors:
                    print(f"   > '{key}' (Vector): [{v1}, {v2}]")
                    found_keys.append(key)

            # 4. DIAGNOSI SPECIFICA
            print(f"\n--- 3. DIAGNOSI CRITICA ---")
            
            # Timecode
            if "timecode" in found_keys:
                print(f"   [OK] TIMECODE trovato.")
            else:
                # Cerchiamo varianti corrotte (es. 0imecode, imecode)
                corrupted = [k for k in found_keys if "imecode" in k or "stamp" in k]
                if corrupted:
                    print(f"   [WARN] Timecode forse corrotto in: {corrupted}")
                else:
                    print(f"   [FAIL] Timecode ASSENTE.")

            # Accel IMU (Per il test fisico)
            if "accel_imu" in found_keys:
                print(f"   [OK] ACCEL_IMU presente!")
            else:
                # Cerchiamo varianti
                candidates = [k for k in found_keys if "accel" in k or "imu" in k]
                if candidates:
                     print(f"   [WARN] 'accel_imu' non trovata. Candidate simili: {candidates}")
                else:
                     print(f"   [FAIL] NESSUNA traccia di accelerometri o IMU.")

            print("\n")

        except KeyboardInterrupt:
            print("Stop.")
            break
        except Exception as e:
            print(f"Loop Error: {e}")
            continue

if __name__ == "__main__":
    main()