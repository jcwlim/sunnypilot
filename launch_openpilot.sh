#!/usr/bin/env bash
sudo date 063010002025
export API_HOST=https://api.konik.ai
export ATHENA_HOST=wss://athena.konik.ai

exec ./launch_chffrplus.sh
