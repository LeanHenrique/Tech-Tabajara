# RTech Tabajara

Projeto desenvolvido para a disciplina **Projeto de Sistemas Robóticos**.

## Equipe

* Lean Henrique Pereira Miranda
* Guilherme Novais de Souza
* Marcus Oliveira
* Luis Eduardo
* Hector Julio Pardinho Soares

## Disciplina

**Projeto de Sistemas Robóticos**

## Sobre

Repositório destinado ao desenvolvimento, documentação e armazenamento dos arquivos do projeto da equipe RTech Tabajara.

## Projeto final: sumô, rouba-bandeira e futebol

### Organização

| Pasta ou arquivo | Conteúdo |
|---|---|
| `projeto-final/` | os três arquivos do torneio: `sumo.py`, `ctf.py`, `soccer.py` (tag `torneio`) |
| `exercicios/<jogo>/<jogo>.py` | versão final de cada jogo, igual à de `projeto-final/` |
| `exercicios/<jogo>/*_v1.py`, `*_v2.py` | versões anteriores; a `_v2` é a usada nas comparações |
| `exercicios/<jogo>/` (outros) | sparrings (réplicas de adversários da liga) e nós dos experimentos de física |
| `exercicios/registros/fase1/` | dados dos experimentos de física do sumô (E1 a E3) |
| `exercicios/registros/final/` | as 30 séries de partidas: placar e eventos (`.json`), log dos nós (`.log`), posições a 10 Hz (`*_frames.json`) e `resumo.csv` |
| `exercicios/videos/` | um vídeo de cada jogo, desenhado a partir das posições gravadas |
| `tools/lockstep.py`, `tools/planos/`, `tools/render.py` | driver das partidas, planos de cada série e gerador dos vídeos |
| `tools/sync.sh`, `tests/` | cópia de `exercicios/` para `projeto-final/` com o verificador do professor; testes offline |
| `relatorio/DECISOES.md` | cada decisão de estratégia com a evidência, inclusive as descartadas |
| `Trabalho Prático 5/` | o kit de simulação (simulador Genesis, ponte ROS e `run.sh`) |

### Rodar um jogo no kit (com ROS, como no torneio)

O kit roda os arquivos da pasta `exercicios/<jogo>/` dele. Copie a versão final e a anterior para lá e use o `run.sh`
(precisa do Docker; a primeira vez leva alguns minutos):

```bash
cp projeto-final/sumo.py   exercicios/sumo/sumo_v2.py     "Trabalho Prático 5/exercicios/sumo/"
cp projeto-final/ctf.py    exercicios/ctf/ctf_v2.py       "Trabalho Prático 5/exercicios/ctf/"
cp projeto-final/soccer.py exercicios/soccer/soccer_v2.py "Trabalho Prático 5/exercicios/soccer/"
cd "Trabalho Prático 5"
./run.sh build                                  # só na primeira vez
./run.sh run sumo robot1:=none                  # sumô contra robô parado
./run.sh run sumo robot1:=sumo_v2.py            # final (hero_0) x v2 (hero_1)
./run.sh run ctf red:=none                      # rouba-bandeira contra time parado
./run.sh run ctf red:=ctf_v2.py                 # final (azul) x v2
./run.sh run soccer red:=none                   # futebol contra time parado
./run.sh run soccer blue:=soccer_v2.py          # v2 (azul) x final (vermelho)
./run.sh down                                   # fecha tudo
```

Antes de enviar uma mudança: `python3 projeto-final/verificar.py` (as mesmas checagens estáticas do torneio).

### Repetir as séries de partidas (sem ROS, acelerado)

`tools/lockstep.py` roda o simulador do kit (mesma física e mesmo juiz) no mesmo processo que os nós dos jogos, com
os arquivos sem alteração. O ROS e a ponte são trocados por um barramento em processo com relógio simulado: o estado
sai a 25 Hz, como no `hero_bridge.py`, e cada nó roda no seu `rospy.Rate` (20 Hz). Não há latência de rede e as
sementes são fixas, então cada placar é uma amostra, não uma taxa.

Ambiente do simulador (Python 3.12, uma vez só), igual ao que o `run.sh build` monta no macOS:

```bash
cd "Trabalho Prático 5"
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python "torch==2.14.1" --index-url https://download.pytorch.org/whl/cpu
uv pip install --python .venv/bin/python --no-deps "genesis-world==1.4.3"
uv pip install --python .venv/bin/python -r docker/sim-requirements.txt
site=$(.venv/bin/python -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")
cp docker/psr_slim_stubs.py "$site/" && echo "import psr_slim_stubs" > "$site/psr_slim_stubs.pth"
cd ..
```

Séries (as 30 levam alguns minutos com 15 processos em paralelo) e vídeos:

```bash
KIT="Trabalho Prático 5"
"$KIT/.venv/bin/python" tools/lockstep.py --kit "$KIT/sim" --utils "$KIT/ros/psr_hero/src" \
    --plan tools/planos/sumo.json tools/planos/ctf.json tools/planos/soccer.json \
    --out exercicios/registros/final --jobs 15
python3 tools/render.py exercicios/registros/final/soccer_final_x_v2_azul_p1_frames.json \
    exercicios/videos/futebol_final_x_v2.mp4 --seconds 90 --title "Futebol: final (azul) x v2 (vermelho)"
```

Use `--only <nome da série>` para rodar uma série só. O `render.py` precisa de Pillow e do `ffmpeg`.

### Resultados das séries (`exercicios/registros/final/resumo.csv`)

Sumô: 20 rodadas por confronto. Rouba-bandeira e futebol: 3 partidas de 5 min por confronto. Placar sempre
"nosso × adversário"; 0 erros nos logs dos nós em todas as séries.

| Jogo | Adversário | Nosso lado | Resultado |
|---|---|---|---|
| Sumô | parado | `hero_0` / `hero_1` | 20 vitórias em 20 / 20 em 20 |
| Sumô | v2 | `hero_0` / `hero_1` | 20 empates por tempo / 20 empates por tempo |
| Sumô | `sumo_tech` (sparring) | `hero_0` / `hero_1` | 20 empates por tempo / 20 empates por tempo |
| Rouba-bandeira | parado | azul / vermelho | 22×0 nas 3 partidas / 22×0 nas 3 partidas |
| Rouba-bandeira | v2 | azul | 3×0, 3×0, 6×0 |
| Rouba-bandeira | v2 | vermelho | 7×0, 6×0, 4×0 |
| Futebol | parado | azul | 29×0, 28×0, 29×0 |
| Futebol | parado | vermelho | 28×0, 29×0, 28×0 |
| Futebol | v2 | azul | 4×0, 4×0, 4×0 |
| Futebol | v2 | vermelho | 5×0, 4×0, 0×0 |

Vídeos (`exercicios/videos/`): `sumo_final_x_parado.mp4` (65 s), `rouba_bandeira_final_x_v2.mp4` (75 s) e
`futebol_final_x_v2.mp4` (90 s).
