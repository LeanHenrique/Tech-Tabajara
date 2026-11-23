# Roteiro do seminário (10 a 12 min), RASCUNHO

> Só números medidos. O que depende do que ainda não foi feito está como `[PREENCHER]`.

## Slide 1: como chegamos às estratégias (1 min)
Código do juiz → física medida → observação da liga → testes contra sparrings. Robustez antes de estratégia: a nota da
competição mede se os códigos **funcionam**.

## Bloco 1: sumô (≈ 3 min)
- **Ideia em uma frase:** encarar e empurrar de frente, com a borda sempre em primeiro lugar.
- **Diagrama:** `ENGAJAR ⇄ BORDA` (ver `relatorio.md`, Seção 3.1).
- **Figura:** [PREENCHER: gif de uma rodada ou gráfico do E2].
- **Resultado principal:** contra parado, +19 vitórias em 120 s nos dois lados; contra quem encara, empate (E2: 10 empates em 10).

## Bloco 2: rouba-bandeira (≈ 3 min)
- **Ideia em uma frase:** defensor plantado sobre a bandeira e atacante que desvia dos oponentes.
- **Diagrama:** estados do defensor e do atacante (Seção 3.2).
- **Figura:** [PREENCHER: gif do ciclo de imunidade do `ctf_tech`].
- **Resultado principal:** contra parado, 9 a 0 nos dois lados; contra defensor plantado, 0 a 0.

## Bloco 3: futebol (≈ 3 min)
- **Ideia em uma frase:** goleiro na linha, um atacante que contorna, alinha e atravessa a bola, e um apoio recuado.
- **Diagrama:** estados do atacante (Seção 3.3).
- **Figura:** [PREENCHER: gif de um gol].
- **Resultado principal:** contra parado, +7 e +6 gols, 0 sofridos; contra a dupla "Nyx", 0 a 0.

## Slide: o que não funcionou (≈ 1 min)
- Inclinar o robô no sumô para arrastar o adversário (E2: 10 empates em 10, deslocamento lateral de 4 cm).
- Empurrão lateral puro (E1: o atacante escorrega e cai em 6 de 6).
- Atacante do futebol ficando parado para sempre (deadlock), achado em partida e por teste de propriedade.
- Réplica de retranca para o Nyx (levou 6 a 0): refeita depois da observação do usuário.

## Slide: resultados no torneio e na liga
[PREENCHER: prints da tabela da liga e dos confrontos contra Tech, Nyx e Claude]

## Banco de perguntas prováveis (respostas curtas)

| Pergunta | Resposta (com a evidência) |
|---|---|
| Por que inclinar o robô no sumô? | Era a hipótese H2. Medimos (E2) e **não funcionou** contra quem encara: 10 empates em 10; descartamos. |
| O que acontece se o adversário também inclinar? | [PREENCHER: não testamos] |
| Por que o defensor fica em cima da bandeira? | No código do juiz, o raio de segurar (0,10 m) é maior que o contato (0,074 m), que é maior que o de pegar (0,07 m). A taxa real de pegada **não foi medida** (E4). [PREENCHER] |
| Por que o goleiro fica na linha e segue a bola em y? | É a v1. A ideia de virar de frente com a bola perto é hipótese e ainda **não está implementada**. [PREENCHER] |
| Como o código sabe de que lado está? | Pelos parâmetros do nó (`~robot`, `~team`, `~role`, `~opponent`); não há id, cor nem sinal de x fixos. Testamos os dois lados de cada jogo. |
| Como vocês garantem que o nó não trava? | Publica `(0, 0)` desde o 1º ciclo, `try/except` em volta da lógica, 63 testes offline (inclusive um teste de propriedade com 300 configurações) e partidas de 2 min dos dois lados. Achamos e corrigimos deadlocks assim. |
| Por que o sumô é tão cheio de empates? | Com a força limitada por eixo, o par frontal anda na média das velocidades-alvo: 0,099 m/s contra parado e ~0 contra quem empurra (E1, E2). |
| Qual a margem de frenagem na borda? | 3,76 cm a 0,2 m/s (E3), por isso `R_SAFE = 0,32 m`. |
