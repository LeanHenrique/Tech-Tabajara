#!/usr/bin/env bash
# Sincroniza os três arquivos de jogo de exercicios/<jogo>/<jogo>.py (onde se desenvolve e se testa
# no kit) para projeto-final/ (o que o torneio lê), confere a cópia, roda o verificar.py e mostra o
# git diff --stat. NUNCA edite projeto-final/ à mão: edite em exercicios/ e rode este script.
#
#   ./tools/sync.sh            copia, verifica e mostra o diff
#   PYTHON=/caminho/python ./tools/sync.sh   escolhe o interpretador (no Windows, o do Git Bash pode ser o da Store)
set -u
cd "$(dirname "$0")/.." || exit 1

# modelo oficial do professor: README.md e verificar.py ficam em projeto-final/ (não são gerados aqui)
mkdir -p projeto-final
for jogo in sumo ctf soccer; do
  src="exercicios/$jogo/$jogo.py"
  [ -f "$src" ] || { echo "✗ falta $src" >&2; exit 1; }
  cp "$src" "projeto-final/$jogo.py"
  chmod +x "projeto-final/$jogo.py" 2>/dev/null || true
  cmp -s "$src" "projeto-final/$jogo.py" && echo "✓ $jogo.py copiado" || { echo "✗ cópia de $jogo.py difere" >&2; exit 1; }
done

PY=${PYTHON:-}
if [ -z "$PY" ]; then
  for c in python3 python; do
    if command -v "$c" >/dev/null 2>&1 && "$c" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)' 2>/dev/null; then PY=$c; break; fi
  done
fi
[ -n "$PY" ] || { echo "✗ nenhum python3 encontrado: use PYTHON=/caminho/python ./tools/sync.sh" >&2; exit 1; }

echo
"$PY" projeto-final/verificar.py
status=$?
echo
git diff --stat -- projeto-final 2>/dev/null
git status --short -- projeto-final 2>/dev/null
exit $status
