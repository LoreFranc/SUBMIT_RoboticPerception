import zmq
import msgpack
import json
import rerun as rr
import math
import sys
from rerun.archetypes import Scalars 
import re


SHOW_3D = True 


# Configurazione
#MADS_ENDPOINT = "tcp://localhost:9091"  #Broker port
MADS_ENDPOINT = "tcp://raspberrypi.local:9091"
TOPIC_FILTER = ["odometry_filter"]        #listening topic

def decode_data(obj):
    if isinstance(obj, dict):
        return {(k.decode('utf-8', errors='ignore') if isinstance(k, bytes) else k): decode_data(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [decode_data(x) for x in obj]
    elif isinstance(obj, bytes):
        try: return obj.decode('utf-8', errors='ignore') 
        except: return obj 
    return obj
    
def get_nested(data, path):
    # 1. Prova accesso diretto (caso flattened)
    if data is None: return None
    if path in data: return data[path]
    #if path.startswith("/") and path[1:] in data: return data[path[1:]]

    # 2. Prova navigazione (caso nested)
    keys = path.strip("/").replace(".", "/").split("/")
    curr = data
    try:
        for k in keys:
            # Gestione indici array (es. "position/0")
            if isinstance(curr, (list,tuple)) and k.isdigit():
                idx = int(k)
                if idx < len(curr): curr = curr[idx]
                else: return None
            elif isinstance(curr, dict):
                if k in curr:
                    curr = curr[k]
                elif k.isdigit() and int(k) in curr:
                    curr = curr[int(k)]
                else:
                    return None
            else:
                return None
        try:
            return float(curr)
        except:
            return None
    except:
        return None

# FUNZIONE CHIAVE: Rimuove caratteri illegali per il JSON
def clean_json_string(s):
    # Rimuove tutti i caratteri di controllo ASCII (0-31) eccetto i whitespace validi per JSON (t, n, r)
    # Questo elimina i vari \x01, \x13 che corrompono le chiavi
    return re.sub(r'[\x00-\x08\x0b\x0c\x0e-\x1f\x7f-\xff]', '', s)


def main():
    # 1. Start Rerun
    print(f"Starting Python visualizer connecting to: {MADS_ENDPOINT}")
    rr.init("MADS_Replication_Python", spawn=True)

    # 2. Connect to MADS (ZeroMQ)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    try:
        socket.connect(MADS_ENDPOINT)
    except Exception as e:
        print(f"Failed to connect to MADS endpoint {MADS_ENDPOINT}: {e}")
        sys.exit(1)
    
    
    # Subscribe to the filter topic
    for topic in TOPIC_FILTER:
        socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        print(f"Subscribed to topic: {topic}")

    # ... (parte iniziale uguale) ...
    print("Waiting for data (DEBUG MODE)...")

    json_decoder = json.JSONDecoder()

    while True:
        try:
            # 1. Receive
            msg = socket.recv_multipart()
            
            # [DEBUG] Stampiamo subito cosa è arrivato a livello grezzo
            print(f"\n[RAW] Ricevuto messaggio Multipart: {len(msg)} frame")
            
            payload = msg[1]

            # 2. Topic Check
            try: 
                topic = msg[0].decode('utf-8')
                print(f"[TOPIC] Decodificato: '{topic}'") # Vediamo cosa legge
            except Exception as e: 
                print(f"[TOPIC ERROR] Impossibile decodificare topic: {msg[0]}")
                continue
            
            if topic not in TOPIC_FILTER: 
                print(f"[SKIP] Topic '{topic}' non è nel filtro {TOPIC_FILTER}")
                continue
            else:
                print(f"[MATCH] Topic '{topic}' accettato!")

            data = None

            # --- DECODIFICA ---
            print(f"[DECODING] Tentativo decodifica payload di {len(payload)} bytes...")
            # Stampa i primi 50 byte per vedere se c'è l'header sporco
            print(f"[PAYLOAD PREVIEW] {payload[:500]}") 

            try:
                # STRATEGIA SANDWICH
                start_idx = payload.find(b'{')
                end_idx = payload.rfind(b'}')
                
                if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
                    json_bytes = payload[start_idx : end_idx+1]
                    json_str = json_bytes.decode('utf-8', errors='ignore')
                    data, _ = json_decoder.raw_decode(json_str)
                    print("[SUCCESS] Decodifica JSON Sandwich riuscita!")
                else:
                    raise ValueError("No JSON boundaries")

            except Exception as e1:
                print(f"[FAIL JSON] Sandwich fallito: {e1}")
                # Fallback MessagePack
                try:
                    unpacker = msgpack.Unpacker(raw=True, strict_map_key=False) 
                    unpacker.feed(payload)
                    data_raw = next(unpacker)
                    data = decode_data(data_raw)
                    print("[SUCCESS] Decodifica MessagePack riuscita!")
                except Exception as e2:
                    print(f"[FAIL MSGPCK] Anche MessagePack fallito: {e2}")
                    continue

            if not isinstance(data, dict): 
                print(f"[ERROR] Il dato decodificato NON è un dizionario: {type(data)}")
                continue

            # 4. Estrazione Dati
            # Se siamo arrivati qui, stampiamo le chiavi principali
            print(f"[DATA KEYS] Chiavi trovate: {list(data.keys())}")
            
            if "debug" in data:
                 print(f"[DEBUG KEYS] Contenuto 'debug': {list(data['debug'].keys())}")

            # ... (Resto del codice di visualizzazione Rerun uguale a prima) ...
            
            time_val = data.get("sim_time")
            if time_val is None: time_val = get_nested(data, "timecode")
            if time_val is not None: 
                 try: rr.set_time_seconds("sim_time", float(time_val))
                 except: pass
           
            if topic == "odometry_filter":
                 # ... (Inserisci qui il resto della logica Rerun che avevi) ...
                 pass # Placeholder per brevità, mantieni il tuo codice di log

        except KeyboardInterrupt:
            print("Stop.")
            break
        except Exception as e:
            print(f"[CRITICAL LOOP ERROR] {e}")
            continue

if __name__ == "__main__":
    main()