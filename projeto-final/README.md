# Projeto final — pasta do torneio

Copiem esta pasta `projeto-final/` para a **raiz** do repositório da equipe (o mesmo dos trabalhos práticos), na
branch padrão (`main` ou `master`). O torneio lê **apenas** estes três arquivos:

```
projeto-final/
├── sumo.py      sumô: um processo por robô (parâmetros ~robot, ~opponent)
├── ctf.py       rouba-bandeira: um processo por robô (~robot, ~team, ~role, ~opponents)
└── soccer.py    futebol: um processo controla os três robôs do time (~team)
```

Os três arquivos aqui são os modelos do kit (os mesmos de `exercicios/<jogo>/` do `psr-hero-kit.zip`). Completem os
TODOs ou substituam pelo código de vocês, mantendo os nomes, a primeira linha `#!/usr/bin/env python3` e a leitura
dos parâmetros.

## Regras (resumo do documento "formato de entrega para o torneio")

- Cada arquivo é autossuficiente: nada de importar outros arquivos da equipe ou ler arquivos do repositório.
- Imports permitidos: biblioteca padrão, `numpy`, `rospy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs` e
  `psr_hero.utils`. Proibidos: `subprocess`, `socket`, `multiprocessing`, `ctypes`.
- A pasta é montada **somente para leitura** no torneio: não gravem arquivos ao lado do código (use `/tmp`, se precisar).
- O mesmo arquivo joga dos dois lados; só publiquem no `cmd_vel` dos robôs do próprio time.

## Antes de enviar

```bash
python3 projeto-final/verificar.py       # mesmas checagens estáticas do torneio
```

No kit, copiem os arquivos para `exercicios/<jogo>/` e testem os dois lados contra um adversário parado:

```bash
./run.sh run sumo robot1:=none     ./run.sh run sumo robot0:=none
./run.sh run ctf red:=none         ./run.sh run ctf blue:=none
./run.sh run soccer red:=none      ./run.sh run soccer blue:=none
```

## Versão usada no torneio

Commit da tag `torneio`, se existir; senão, o último commit da branch padrão até **25/11/2026, 23:59**.

```bash
git tag torneio && git push origin torneio            # marcar
git tag -f torneio && git push -f origin torneio      # mover para um commit mais novo
```

Versões anteriores, registros, vídeos e o relatório podem ficar em outras pastas (ou aqui, com outros nomes).
