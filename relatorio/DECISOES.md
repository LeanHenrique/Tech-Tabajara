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
