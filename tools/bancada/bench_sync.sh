#!/usr/bin/env bash
# Bancada de testes: ~/psr-hero-kit (kit no WSL) recebe por cima os jogos e ferramentas do repositório da equipe.
TEAM=${TEAM:-/mnt/c/Users/marcu/OneDrive/Documentos/Robotica/Tech-Tabajara}  # AJUSTE para o caminho do clone (no WSL: /mnt/c/...)
KIT=~/psr-hero-kit
[ -d "$KIT" ] || { echo "kit não existe em $KIT" >&2; exit 1; }
for j in sumo ctf soccer; do
  cp "$TEAM"/exercicios/$j/*.py "$KIT"/exercicios/$j/ 2>/dev/null
done
mkdir -p "$KIT"/exercicios/tools "$KIT"/exercicios/registros
cp "$TEAM"/exercicios/tools/*.py "$KIT"/exercicios/tools/
chmod +x "$KIT"/exercicios/*/*.py "$KIT"/exercicios/tools/*.py 2>/dev/null
echo "bancada sincronizada: $(ls "$KIT"/exercicios/sumo | tr '\n' ' ')"
