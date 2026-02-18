#!/bin/bash
# if broker does not start because it can't find mads.ini do:
# sudo rm /usr/local/etc/mads.ini
# sudo ln -sf $(pwd)/mads.ini /usr/local/etc/mads.ini



# Configurazione
MADS_PATH="." # Siamo già nella cartella giusta

echo "Avvio Simulazione MADS Completa..."

# 1. Pulisci vecchi processi
echo "Pulizia processi precedenti..."
pkill -f mads
pkill -f rerun
sleep 1

# 2. Lancia il Visualizzatore Python (in background)
echo "   -> Avvio Rerun Sink..."
    mads sink rerun.plugin -o endpoint=tcp://raspberrypi.local:9091 -o topics=odometry_filter &
    SINK_PID=$!
sleep 2 # Aspetta che parta

# 4. Lancia il Filtro (Odometria)
echo "Avvio Filtro Odometria..."
mads filter odometry_filter/build/odometry_filter.plugin -s tcp://raspberrypi.local:9092 &
FILTER_PID=$!

# Funzione per chiudere tutto quando premi Ctrl+C
cleanup() {
    echo ""
    echo "Arresto simulazione..."
    kill $VIS_PID $BROKER_PID $FILTER_PID $SRC_ENC_PID $SRC_IMU_PID $SRC_HTC_PID 2>/dev/null
    pkill -f mads
    exit
}

# Intercetta Ctrl+C
trap cleanup SIGINT

echo "Tutto avviato! Premi Ctrl+C per terminare."
wait
