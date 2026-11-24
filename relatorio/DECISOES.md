# Decisões de estratégia (RTech Tabajara, projeto final)

Uma entrada por decisão, no momento em que ela é tomada. Formato:
decisão · por quê · evidência · resultado · status (adotada, descartada ou experimental).
Medidas vêm de `exercicios/registros/` (experimentos da Fase 1: `registros/fase1/`).

---

## [sumô] A força é limitada por eixo do mundo, e o par em contato anda na média das velocidades-alvo — 23/11/2026
- **Decisão:** modelar o contato como: cada robô aplica no máximo 0,20 N em cada eixo x e y do mundo; o par tende à
  média das velocidades-alvo. Empurrar um robô parado de frente funciona; contra um robô que também empurra de
  frente, empata.
- **Por quê:** é como o simulador controla o robô (`control_dofs_velocity` com alvo `(v·cosψ, v·sinψ, ω)` e ganho
  `KV_LIN = 3,0`, força limitada a `FORCE_MAX = 0,20` N por eixo).
- **Evidência:** `sim/psr_sim/world.py` (linhas 290 a 299 e 407 a 412). Experimento **E1 frontal**
  (`registros/fase1/exp_push_front.csv`, 6 rodadas): velocidade do par em contato **0,099 m/s** (0,097 a 0,100), igual à
  média de 0,2 e 0; o robô parado caiu em **6 de 6** rodadas, em ~2,25 s, deslocado ~22 cm.
- **Resultado:** confirma o mecanismo com adversário **parado**. **Falta** o caso de dois robôs ativos (H1: empate
  frontal; H2: força lateral perto dos eixos), que é o E2.
- **Status:** adotada (parcial).

## [sumô] Margem da borda: a frenagem a 0,2 m/s consome cerca de 3,8 cm — 23/11/2026
- **Decisão:** manter `R_SAFE` inicial em 0,34 m (a perda é com o CENTRO acima de 0,40 m) e **calibrá-lo** com os
  dados dos próximos experimentos. Contar na margem ~4 cm só de frenagem.
- **Por quê:** o briefing supunha ~1 cm; a medida é maior.
- **Evidência:** **E3** (`registros/fase1/exp_brake.csv`, 3 tentativas por velocidade, ida e volta): distância de
  frenagem **3,76 cm** a 0,20 m/s (3,50 a 3,89) e **1,57 cm** a 0,10 m/s; parada total em ~0,45 s. Só a física
  (força limitada, massa 0,15 kg) daria ~1,5 cm a 0,2 m/s. O que sobra (~2,3 cm) equivale a uns **115 ms** de latência
  comando-efeito-odometria. **Esse valor de latência é dedução** a partir dos dois pontos, não foi medido à parte.
- **Resultado:** margem de ~6 cm entre 0,34 e 0,40, contra ~3,8 cm de frenagem: folga de ~2 cm, pequena.
- **Status:** experimental (calibrar no E2).

## [sumô] Empurrão puramente lateral é instável e foi descartado como tática — 23/11/2026
- **Decisão:** não usar empurrão lateral puro. O "arrasto" deve vir de um empurrão **frontal com o heading inclinado**
  (estado `ARRASTAR`), mantendo o contato pela frente.
- **Por quê:** no empurrão lateral o par anda devagar e a linha de contato (pelos centros) gira; o atacante
  escorrega para o lado do adversário e sai do ringue antes dele.
- **Evidência:** **E1 lateral** (`registros/fase1/exp_push_side.csv`, 6 rodadas): velocidade do par **0,04 m/s**
  (0,03 a 0,05), bem abaixo dos 0,10 m/s do frontal e abaixo da faixa de 0,07 a 0,13 m/s suposta no briefing.
  O adversário foi deslocado ~25 cm em y, mas caiu em **0 de 6** rodadas; **o atacante caiu em 6 de 6**.
- **Resultado:** a resistência lateral é **maior** que a estimada, e o contato lateral não é estável nesta montagem.
  Limite do experimento: o alinhamento do atacante era simples (sem correção fina do deslocamento em x).
- **Status:** descartada (lateral puro). H2 (inclinação perto dos eixos) continua a testar no E2.

## [sumô] Contra um adversário que encara, empurrar de frente empata e inclinar o heading não ajuda — 23/11/2026
- **Decisão:** **abandonar o `ARRASTAR` como forma de vencer um adversário que encara e empurra** (o "facer"). Contra
  ele, o resultado esperado é empate; o ganho vem de **não perder** (`BORDA`, `ANULAR`) e de **aproveitar erros**
  do adversário (oportunismo), não de uma técnica de arrasto.
- **Por quê:** com as duas forças saturadas, o par não se move (H1), e a força lateral extra (H2) não aparece quando o
  adversário gira para acompanhar.
- **Evidência:** **E2** (`registros/fase1/exp_tilt_00.csv` e `exp_tilt_35.csv`), hero_0 inclinado contra o `exp_facer`
  (hero_1), 10 rodadas cada:

  | φ do atacante | vitórias | empates | derrotas | deslocamento lateral do adversário (média) |
  |---|---|---|---|---|
  | 0° (frontal) | 0 | 10 | 0 | 0,4 cm |
  | 35° | 0 | 10 | 0 | 4,0 cm (em ~43 s de contato) |

  Todos os empates foram por tempo (40 s). A inclinação foi escolhida a cada rodada para levar o adversário para
  longe do centro.
- **Resultado:** **H1 confirmada** (empate frontal). **H2 não confirmada** em φ = 35° contra o facer. Isso também
  explica a liga: líderes do sumô têm muitos empates (briefing, Seção 4).
- **Limites:** só φ = 0° e 35° foram medidos; φ = 20° e 50° ficaram pendentes. O facer é meu, não o Tech real.
  Não testei o `ARRASTAR` contra um adversário que **não** acompanha (parado, ou de costas).
- **Status:** descartada (como arma contra adversário que encara). Aberta para adversário que não encara.

## [ctf] Defensor plantado na bandeira: tag 0,10 > contato 0,074 > pegada 0,07 — 23/11/2026
- **Decisão:** defensor plantado exatamente sobre a bandeira, com posição rígida; o atacante só pega se o deslocar
  uns 4 mm ou mais.
- **Por quê:** o raio de segurar (0,10 m) é maior que o contato entre carcaças (0,074 m), que é maior que o de pegar
  a bandeira (0,07 m): o atacante é segurado antes de encostar.
- **Evidência:** `sim/psr_sim/scenarios/ctf.py` (constantes de pegar, segurar e pontuar). **Ainda não medido** (E4).
- **Status:** adotada (premissa do código; falta medir a taxa de pegada).

## [ctf] Ciclo de imunidade do Tech como adversário de referência — 23/11/2026
- **Decisão:** o sparring `ctf_tech` replica o comportamento observado: defensor plantado que **encara** o intruso;
  atacante que cola no defensor, é segurado 4 s, fica imune 1,5 s, empurra o defensor e foge de ré.
- **Evidência:** observação da liga em 23/11, confirmada pelo usuário (briefing, Seção 4). Não há registro gravado.
- **Status:** adotada como premissa (a réplica ainda não existe).

## [futebol] Goleiro de frente quando a bola está perto — 23/11/2026
- **Decisão:** goleiro de lado para a bola (deslizando em y) quando ela está longe, e virado de frente para o
  campo quando ela está perto da área.
- **Por quê:** de lado, um robô resiste mal a um empurrão (H3) e é levado para dentro do gol junto com a bola.
  O E1 mostrou que a resistência lateral **não é desprezível** (0,04 m/s do par), então o argumento precisa de teste.
- **Status:** experimental.

## [geral] Gerência por placar e saldo — 23/11/2026
- **Decisão:** o desempate é por saldo, depois pontos marcados, depois confronto direto. Empatado ou contra
  adversário que não ameaça: continuar atacando. Vantagem curta contra adversário perigoso no fim: fechar.
- **Evidência:** `formato_torneio.pdf`, Seção 7.
- **Status:** adotada.

## [futebol] Retranca total (três na linha do gol) aceita pelo professor — 23/11/2026
- **Decisão:** manter a retranca como constante ligável (`PARK_BUS`), documentando quando ela entra.
- **Evidência:** resposta do usuário no briefing (aceita pelo professor).
- **Status:** adotada (a implementar).

---

# Fase 3: sparrings (23/11/2026)

Sparrings criados em `exercicios/<jogo>/`: `sumo_facer`, `sumo_dodger`, `ctf_tech`, `ctf_camper`, `soccer_swarm`,
`soccer_nyx` (e as cópias congeladas `sumo_v1`, `ctf_v1`, `soccer_v1`). Todos sobem sem erro. Resultados de uma
rodada curta (60 a 100 s) da nossa v1 contra cada um, **sem repetição**:

## [sumô] A v1 perde para quem desvia na largada e ataca a lateral — 23/11/2026
- **Decisão:** a v2 precisa de uma resposta ao desvio (reorientar rápido para o flanco do adversário e não oferecer a
  lateral); a v1 só encara a posição atual do adversário.
- **Evidência:** v1 (hero_0) contra `sumo_dodger`: **1 vitória e 2 derrotas** em 3 rodadas; contra `sumo_facer`:
  2 empates (esperado, H1). Sem erros nos logs.
- **Limite:** 3 rodadas é pouco; é um indício, não uma taxa. O `sumo_dodger` é invenção minha a partir do briefing.
- **Status:** experimental (a confirmar com mais rodadas na Fase 4).

## [ctf] A réplica do Tech funciona e chega a roubar a bandeira; a v1 não fura defensor plantado — 23/11/2026
- **Decisão:** manter o `ctf_tech` como principal sparring do CTF; o defensor da v2 precisa de **antecipação** (perseguir
  ao primeiro sinal de pegada), e o atacante da v2 precisa de uma resposta ao defensor plantado.
- **Evidência:** v1 (azul) contra `ctf_tech`: placar **0 a 0** em 100 s, com o log mostrando
  `APROXIMAR -> PRESO -> EMPURRAR -> FUGA_RE` e o atacante vermelho **carregando a nossa bandeira** ao final (nosso
  defensor correu atrás); nosso atacante foi segurado 16 vezes. Contra `ctf_camper`: 0 a 0 (ninguém fura ninguém).
- **Limite:** o `ctf_tech` é réplica minha do relato do briefing, não o código real do Tech.
- **Status:** adotada como sparring.

## [futebol] A réplica do Nyx NÃO representa o Nyx real — 23/11/2026
- **Decisão:** não tirar conclusões sobre retranca a partir do `soccer_nyx`.
- **Evidência:** a v1 marcou **6 a 0** contra a réplica em ~95 s, enquanto o Nyx real tem 15 gols a favor e 0 contra em
  28 jogos na liga. Contra `soccer_swarm`: 0 a 0 em ~57 s.
- **Pendente:** formação real do Nyx (onde ficam os três robôs e quando alguém sai), que o usuário vai observar na
  liga. Depois disso, ajustar o `soccer_nyx`.
- **Status:** experimental.

---

# Fase 3 (revisão): adversários reais descritos pelo usuário, 23/11/2026

O usuário observou os dois adversários na liga e descreveu o comportamento. Os sparrings foram refeitos com isso.

## [futebol] Formação do Nyx: goleiro no gol e dois jogadores de linha em dupla — 23/11/2026
- **Observação (usuário):** um robô fica no gol; os outros dois vão em direção à bola, um com a bola e o outro um pouco
  afastado, e eles se alternam.
- **Decisão:** `soccer_nyx` reescrito (goleiro, atacante com a bola e apoio 0,28 m atrás do atacante, papéis trocando
  pelo tempo de chegada). Substitui a réplica antiga de retranca com três na linha do gol, que estava errada (levou 6 a 0).
- **Evidência:** nossa v1 (azul) contra o novo `soccer_nyx`: **0 a 0** em ~100 s, compatível com o Nyx real (0 sofridos em
  28 jogos). Uma rodada, sem repetição.
- **Consequência para a v2:** o apoio **seguir o atacante** (rebote) em vez de esperar parado; a retranca de três
  na linha do gol não é o que o Nyx faz.
- **Status:** adotada como sparring.

## [sumô] Tech: larga a toda em direção ao alvo, empurra até a borda e, se detecta a borda com o inimigo atrás, vira e sai — 23/11/2026
- **Observação (usuário):** o Tech larga à velocidade máxima em direção ao alvo e vai empurrando até a borda; ao
  empurrar, fica atrás do adversário. Se detecta a borda e o inimigo ficou mais atrás, **vira e sai da borda**.
- **Decisão:** `sumo_tech` criado com essa regra (sem ré). Ele empata com um `ENGAJAR` puro, como a liga mostra
  (10V 17E 1D).
- **Evidência:** nossa v1 (hero_0) contra `sumo_tech`: **2 empates** em 100 s, 0 vitórias e 0 derrotas.
- **Oportunidade para a v2 (hipótese, ainda não testada):** quando o Tech vira para sair da borda, ele gira e
  **oferece a lateral ou as costas**; o `Oportunismo` do briefing (empurrar quando o adversário gira ou está perto da
  borda) deve aproveitar esse momento.
- **Status:** adotada como sparring.

---

# Fase 4: v2 (23/11/2026)

## [sumô] Em contato, a BORDA não pode recuar: era a causa das derrotas da v1 — 23/11/2026
- **Decisão:** na v2 a `BORDA` só vale **sem contato** (distância entre centros ≥ 0,10 m). Em contato, o robô mantém o
  empurrão (`ENGAJAR`).
- **Por quê:** com as duas forças saturadas e opostas a força líquida é zero e o par **mantém a velocidade que tem**.
  Quando o par chegava à borda do adversário (e à nossa, r > 0,32), a `BORDA` mandava a v1 dar ré para o centro:
  largava o empurrão, e o adversário nos levava através do ringue até o nosso lado.
- **Evidência:** gravação a 10 Hz (`registros/fase4/v1_vs_dodger_a.txt`, 12 rodadas, `tools/fase4_sumo.py`). Na rodada 7,
  de `rt = 2` a `5` o par andava a +0,06 m/s para a borda do adversário (ele em r = 0,39); em `rt = 5,0` a nossa `BORDA`
  disparou (r = 0,32) e a velocidade do par **inverteu** para −0,12 m/s até caírmos em `rt = 11,6`. As 3 derrotas da v1
  foram nesse padrão, todas em rodadas longas (13 a 28 s). As 2 derrotas do teste curto anterior (3 rodadas) eram ruído.
- **Resultado (antes × depois, contra `sumo_dodger`):** v1 **5V 3D 4E** em 12 rodadas; v2 **31V 0D 0E** em 31 rodadas
  (`registros/fase4/v2_vs_dodger_a.txt`). Contra `sumo_tech` (nos dois lados): só empates, 0 derrotas, igual à v1.
  Contra parado (simulador limpo): +10 vitórias em ~64 s nos dois lados, o mesmo ritmo da v1.
- **Limite:** o `sumo_dodger` é derivado da v1 e **tem o mesmo defeito**; por isso perde para quem não recua em contato. O 31
  a 0 não prova que a v2 vença o Tech real. Contra quem encara sem esse defeito, o resultado continua sendo empate (H1).
- **Status:** adotada.

## [rouba-bandeira] O atacante usa o `held` e a imunidade de 1,5 s para ir direto — 23/11/2026
- **Decisão:** a v2 rastreia o `held` do próprio robô (do `/game/state`). Preso: fica parado. Solto: imune por 1,5 s,
  vai **direto** no defensor plantado (sem desviar nem girar). Com a bandeira e defensor plantado: foge de ré de frente
  para ele. Com o defensor longe da bandeira, mantém o desvio por campo repulsivo da v1.
- **Por quê (observação do usuário na liga):** depois de ser segurado, o atacante da v1 girava (repulsão dos oponentes e
  detector de travamento) e voltava a ser segurado.
- **Evidência (v1 × v2 contra a réplica `ctf_tech`):** azul 0 a 0 → **2 a 0** em 100 s; vermelho **2 a 0** em 80 s; 0 pontos
  sofridos nos dois lados. Contra parado (simulador limpo), azul +5 em 68 s e vermelho com um ponto a cada ~15 s.
- **Limite:** o `ctf_tech` é réplica minha do relato do briefing, não o Tech real. Uma execução por lado.
- **Status:** adotada.

## [futebol] Apoio que acompanha o atacante e goleiro que prevê a bola — 23/11/2026
- **Decisão:** o apoio não espera parado: com a bola no nosso campo (> 0,25 m do centro) cobre o ponto entre a bola e o
  nosso gol; com a bola no ataque acompanha o atacante, 0,30 m atrás dele e 0,12 m de lado. O goleiro usa a velocidade
  da bola para ficar onde ela vai cruzar a linha (só se vem para o nosso gol a mais de 0,03 m/s e em até 3 s).
- **Por quê (observação do usuário na liga):** na v1 o apoio parado era empurrado por um jogador adversário e o goleiro
  "ia mais ou menos" (só seguia a posição da bola). O Nyx real faz o apoio acompanhar.
- **Evidência:** contra `soccer_nyx` (nos dois lados) e `soccer_swarm`: **0 a 0** em todos, igual à v1: a mudança não
  piorou nem sofreu gols, mas **não gerou gols** contra adversários defensivos. Contra parado (simulador limpo): azul
  5 a 0 e vermelho 4 a 0 em 71 s, ritmo igual ao da v1.
- **Limite:** a v2 **não foi testada contra o Claude real** (só descrito pelo usuário). O goleiro preditivo e o apoio têm
  testes offline, mas não há medida isolada do ganho de cada um.
- **Status:** adotada (ganho no jogo real ainda não medido).

## [método] Teste de partida só vale com o simulador reiniciado — 23/11/2026
- **Observação:** duas "regressões" apontadas (futebol 0 a 0 e CTF vermelho 0 pontos contra parado) **desapareceram** ao
  reiniciar o simulador. O time "parado" fica onde o jogo anterior o deixou e pode bloquear a bola ou a bandeira.
- **Decisão:** os testes contra adversário parado usam simulador reiniciado antes de cada partida.
- **Status:** adotada.
