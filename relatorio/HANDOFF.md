# Passagem de bastão: projeto final PSR (RTech Tabajara)

Escrito em 23/11/2026, no fim de uma sessão longa de trabalho. Leia este arquivo antes de mexer em qualquer coisa.
O briefing original está em `ESP/BRIEFING_CLAUDE_CODE.md` na máquina do Marcus (fora deste repositório).

## 1. Prazos (não passam)

| O quê | Quando |
|---|---|
| **Tag `torneio`** (ou o último commit da branch padrão) | **25/11/2026, 23:59** (o briefing sugere fazer até ~22h) |
| Competição | 26/11/2026, 19:00 a 20:40 |
| **Relatório PDF (Moodle) e seminário (10 a 12 min)** | **26/11/2026** |

Criar a tag: `git tag -f torneio && git push -f origin torneio`. **Com a tag, a liga e o torneio ignoram os pushes
seguintes** até ela ser movida. Não crie a tag antes de estar satisfeito, e depois de criada confira na página da liga
que o commit em execução é o da tag.

## 2. O que está na liga agora (commit `49ca39f`)

As três **v2**, em `projeto-final/` (`sumo.py`, `ctf.py`, `soccer.py`). Todas passaram: `projeto-final/verificar.py`, 52+
testes offline (`tests/`), e partidas de 1 a 2 min dos dois lados de cada jogo contra adversário parado, contra a própria
versão e contra os sparrings, sem exceção nos logs. **A nota da competição (10 pts) avalia se os três códigos
funcionam, não a colocação.** Se algo novo ficar instável, a regra de corte é: volte para a última versão que funcionou
(os arquivos congelados `*_v1.py` e `*_v2.py` em `exercicios/<jogo>/` servem para isso).

Versões congeladas (todas em `exercicios/<jogo>/`): `sumo_v1/v2`, `ctf_v1/v2`, `soccer_v1/v2`. A v2 é a que está na liga.

## 3. Como trabalhar (fluxo)

- **Edite só em `exercicios/<jogo>/<jogo>.py`.** Depois rode `./tools/sync.sh` (copia para `projeto-final/`, confere com `cmp`
  e roda o `verificar.py` do professor). **Nunca edite `projeto-final/` à mão.** No Windows o `python3` do Git Bash pode ser o
  da Store: use `PYTHON=/caminho/python ./tools/sync.sh`.
- **Testes offline** (sem ROS): `python tests/test_projeto_final.py`, `python tests/test_v1.py`, `python tests/test_tools.py`.
  Eles carregam o arquivo de **`projeto-final/`**, então rode o `sync.sh` antes (senão testam a versão antiga).
- **Antes de cada push:** `verificar.py` sem ✗, testes verdes, e a versão rodou **nos dois lados** no simulador. Fins de linha
  LF (0 bytes CR nos arquivos de jogo) e bit de executável (`git update-index --chmod=+x`).
- **Commits sem o trailer `Co-Authored-By: Claude`** (pedido explícito do Marcus).
- Autor local configurado neste clone: "Marcus Oliveira".

## 4. Ambiente de teste (leia, é fácil errar)

- Roda em **WSL2 + Docker Desktop** (integração com o Ubuntu ligada). O kit original fica em `~/psr-hero-kit` no WSL (clone
  do repositório do TP5, `marcusoliveira57/Tp5-Hero`). **O kit não está neste repositório.**
- **Scripts de bancada** em `tools/bancada/` (copiados para cá no fim da sessão; antes estavam numa pasta temporária):
  `bench_sync.sh` (copia jogos e ferramentas deste repo para `~/psr-hero-kit/exercicios/`), `play.sh` (uma partida de teste
  e leitura do placar), `watch*.sh` (imprime poses de bola e robôs), `rec_sumo.sh` e `run_exp.sh`. Eles têm o caminho do clone
  fixo em `TEAM` (padrão `/mnt/c/Users/marcu/...`): **ajuste a variável `TEAM`** na sua máquina.
  Exemplo: `FRESH=1 bash tools/bancada/play.sh soccer 70 "azul contra parado" red:=none`.
- **`FRESH=1` é obrigatório nos testes contra adversário parado.** O simulador fica aberto entre partidas e o time "parado" fica
  **onde o jogo anterior o deixou**, bloqueando a bola ou a bandeira. Isso gerou DOIS falsos alarmes de regressão nesta
  sessão (futebol 0 a 0 e CTF vermelho 0 pontos). Sempre reinicie o simulador.
- **Janela 3D no WSL:** o `run.sh` do TP5 foi alterado para montar `/tmp/.X11-unix` (o caminho padrão do kit não funcionava).
  Sem isso aparece "No display detected".
- **Docker Desktop não subia** por um socket velho e travado em `%LOCALAPPDATA%\docker-secrets-engine\engine.sock`
  (erro `initializing Secrets Engine ... rename`). A correção foi **renomear a pasta** `docker-secrets-engine` para
  `docker-secrets-engine.old-20261123` e reabrir. **Nunca use "Reset to factory defaults"**: apagaria as imagens e um
  container de outro projeto do Marcus.
- Ferramentas locais (não entram nos arquivos de jogo, que não podem abrir arquivos): `exercicios/tools/` (`log_game.py`,
  `summarize.py`, `watch_soccer.py`, `watch_sumo.py`, `watch_ctf.py`, `fase1_analise.py`, `fase4_sumo.py`).
  **Armadilha de teste:** `compute_command falhou` e `compute_commands falhou` são mensagens diferentes; conte por `falhou`.

## 5. Adversários e sparrings (`exercicios/<jogo>/`)

Todos são **réplicas minhas** a partir de descrições, nunca o código real. Nomes: `sumo_facer`, `sumo_dodger`, `sumo_tech`,
`ctf_tech`, `ctf_camper`, `ctf_sticky`, `soccer_swarm`, `soccer_nyx`.

O que o Marcus observou na liga (fonte das réplicas):
- **Tech, sumô:** larga a toda e empurra até a borda ficando atrás do adversário; se detecta a borda e o inimigo ficou atrás,
  vira (sem ré) e sai.
- **Tech, CTF:** defensor plantado que encara; atacante com ciclo de imunidade (segurado 4 s, imune 1,5 s, empurra o defensor,
  pega a bandeira e foge de ré de frente para ele). Em outra observação, a defesa do Tech **empurra o atacante antes de ele
  pegar a bandeira** (defensor ativo).
- **TuoLing, CTF:** defensor mais no meio do campo dele, **sempre cola** no nosso robô; o atacante empurra o nosso defensor
  plantado para fora do círculo e rouba a bandeira. O **Claude** (time do professor) venceu o TuoLing contornando o defensor, e
  quando fica de costas para a bandeira e de frente para o defensor, **dá ré até a bandeira**, pega, desvia e leva para o campo dele.
- **Nyx, futebol:** goleiro no gol e dois de linha em dupla, um com a bola e o outro um pouco atrás, trocando de papel.
- **R2D2, futebol (o melhor da liga):** empurra a bola com velocidade e os nossos jogadores abriam caminho; o nosso goleiro
  buscava o meio do gol e deixava a bola entrar; o nosso atacante dava "toquinhos" sem velocidade; o jogador mais longe da bola
  ficava errático.
- **R2D2, sumô:** os dois colidem no centro e o nosso empurra até ele sair, várias vezes. Antes da v2 estávamos em 4º.
- **Resultado da liga relatado pelo Marcus depois da v2:** sumô com duas vitórias por 3 a 0, uma delas contra o Claude.

## 6. O que foi medido (resumo; detalhes e números em `DECISOES.md`)

- **Física (`registros/fase1/`):** frenagem 3,76 cm a 0,2 m/s (E3); empurrão frontal move o par a 0,099 m/s (E1); lateral é
  instável (o atacante escorrega e cai); contra quem encara, empurrar de frente **empata** e inclinar o heading **não ajuda**
  (E2, só φ = 0° e 35°).
- **Princípio que explica muita coisa:** com a força limitada a 0,20 N **por eixo do mundo**, duas forças opostas saturadas
  somam zero e o par **mantém a velocidade que tem**. Quem decide é o momento no impacto, não a força.
- **Sumô v2:** `BORDA` só sem contato (a v1 recuava em contato e perdia partidas ganhas). Contra `sumo_dodger`: 5V 3D 4E em 12
  rodadas (v1) para 31V 0D em 31 (v2). Cuidado: o `dodger` tem o mesmo defeito da v1.
- **CTF v2:** atacante usa o `held` e a imunidade de 1,5 s. Contra a réplica `ctf_tech`: 0 a 0 para 2 a 0 nos dois lados.
- **Futebol v2:** apoio acompanha o atacante (ou cobre o gol) e goleiro prevê onde a bola cruza a linha. Contra Nyx e swarm
  (réplicas): 0 a 0, igual à v1. Foram corrigidos três deadlocks do atacante (ver `DECISOES.md` e o teste de propriedade).

## 7. Experimentos que NÃO foram para a liga (resultado negativo, estão no repositório)

| Arquivo | O que tentou | Por que não foi |
|---|---|---|
| `ctf/ctf_v3_contornar.py` (+ `tests/experimental_ctf_v3.txt`) | tática do Claude: contornar o defensor ativo, encarar e dar ré até a bandeira | contra `ctf_sticky`: 0 a 0, ~18 vezes segurado. O defensor perfeito nos alcança em ~3 s. Pode funcionar contra o TuoLing real |
| `soccer/soccer_v3_apoio.py` (+ `tests/experimental_soccer_v3_apoio.txt`) | apoio com lado travado e sem ré | contra parado: 3 e 3 gols em 68 s (v2: 5 e 4). Sem ganho |
| (removido) condução da bola | atacante persegue a bola em vez de voltar ao ponto de preparação | azul fez 2 gols em 67 s (v2: 5). Piorou |

## 8. Pendências, em ordem sugerida

1. **Confirmar na página da liga** que as três v2 estão "no padrão" e sem traceback nos logs.
2. **Tag `torneio`** (Seção 1). Faça com folga.
3. **Relatório PDF e slides**: rascunhos em `relatorio/relatorio.md` e `relatorio/slides.md` (só números medidos; o resto é
   `[PREENCHER]`). Falta incluir a Fase 4 completa, os resultados da liga com prints, e converter os diagramas Mermaid.
4. **Vídeos** (1 a 2 min por jogo): nenhum foi gravado.
5. Experimentos não feitos: **E4** (taxa de pegada no CTF) e **E5** (chute no futebol); E2 com φ = 20° e 50°.
6. Ideias não implementadas: gerência por placar e saldo (desempate é por saldo), retranca ligável no futebol, defensor do CTF
   com antecipação (reagir ao `held` do atacante adversário), goleiro que corta o ângulo.
7. Futebol: o que o R2D2 faz de melhor ainda não foi entendido nem medido. Provavelmente é o maior ganho disponível.

## 9. Riscos conhecidos

- Os resultados são de **uma execução** de 60 a 150 s por situação; o futebol tem ruído grande (gols contra parado variaram de
  2 a 7 entre execuções parecidas).
- O goleiro cobre 7,4 cm de um gol de 40 cm e anda no máximo a 0,2 m/s: contra chutes fortes há limite físico.
- Os arquivos de jogo **não podem abrir arquivos** (a pasta é somente leitura no torneio) nem importar outros arquivos da equipe.
- O `readme.md` da raiz é dos colegas (TPs 1 a 5): não foi tocado. Este repositório só **acrescentou** pastas na raiz.
