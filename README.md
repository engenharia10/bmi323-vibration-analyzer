# BMI323 Vibration Analyzer — ESP32-S3

Analisador de vibração com IMU **Bosch BMI323** e **ESP32-S3**. O firmware lê o
acelerometro pelo FIFO em alta taxa, calcula a FFT no proprio ESP32 e serve uma
interface web (pasta `data/`, LittleFS) que mostra o espectro, identifica as
frequências prováveis de oscilação e permite **projetar filtros** (notch e
passa-baixa) vendo o efeito em tempo real — cru vs. filtrado no mesmo gráfico.

```
   BMI323  --SPI-->  ESP32-S3  --WiFi/WebSocket-->  navegador
   (FIFO)            FFT + filtros IIR              gráficos + projeto de filtro
```

## Placas

`platformio.ini` tem dois ambientes; o padrão é a Waveshare.

### `waveshare-s3-touch-7` — Waveshare ESP32-S3-Touch-LCD-7 (800×480, N8R8)

BMI323 no **conector I²C (header P4)**, que é o mesmo barramento GPIO8 (SDA) /
GPIO9 (SCL) do touch GT911 e do expansor CH422G. Não há conflito de endereço
(BMI323 em 0x68 ou 0x69), mas o clock fica em **400 kHz** porque o GT911 não
acompanha Fm+.

A placa programa por uma ponte **USB-UART CH343**, então `ARDUINO_USB_CDC_ON_BOOT=0`.

**Teto do I²C medido nesta placa**, com o giroscópio ligado (6 words por quadro):

| ODR configurado | taxa real | veredito |
|---|---|---|
| 800 Hz | 794 Hz | folga grande |
| **1600 Hz** | **1589 Hz** | **padrão**, ~50 % do barramento |
| 3200 Hz | 2484 Hz | satura, perde amostra |

Se um dia o touch GT911 passar a ser usado no mesmo barramento, volte para
800 Hz. O firmware avisa no serial (`BARRAMENTO SATURADO`) sempre que a taxa
real cair abaixo de 90 % da configurada — com o barramento saturado a escala
de frequência do espectro fica errada.

### `devkit-spi` — placa genérica com o BMI323 no SPI

Padrão de pinos em `include/config.h`:

| BMI323 | ESP32-S3 |
|--------|----------|
| SCK / SCL  | GPIO 12 |
| SDO / SDA  | GPIO 13 (MISO) |
| SDI / MOSI | GPIO 11 |
| CS         | GPIO 10 |
| VDD/VDDIO  | 3V3 |
| GND        | GND |

SPI chega a 6.4 kHz de ODR sem apertar o barramento. O endereço I²C
(0x68 ou 0x69, conforme o nível do pino SDO) é **detectado sozinho** no boot.

### Interface pela USB (Web Serial)

Quando o PC não está na mesma rede da placa, o botão **serial** na barra de
status abre a porta USB direto do navegador e recebe os mesmos pacotes do
WebSocket — espectro, formas de onda, atitude e configuração, tudo funcional.

Requisitos: Chrome ou Edge, e a página servida por `http://localhost` ou
`http://127.0.0.1` (o Web Serial exige contexto seguro; abrir o arquivo por
`file://` não funciona). Medido nesta placa: **77 kB/s** a 2 Mbaud, com os
mesmos 5 pacotes de espectro por segundo do WiFi.

Abrir a porta **reinicia a placa** — o circuito de auto-reset do ESP32 é
acionado pelo DTR/RTS que o navegador levanta. O boot leva alguns segundos
(WiFi, LittleFS, varredura I²C), então o pedido de streaming é repetido até o
primeiro quadro chegar; a barra mostra o progresso.

O streaming fica desligado por padrão — senão o monitor serial viraria uma
sopa de binário. O navegador liga sozinho ao conectar; no monitor, `stream 1`
liga e `stream 0` desliga.

Formato do quadro: `AB CD <len:u16> <payload> <xor>`. O payload é o mesmo do
WebSocket: começando com `0xA5` é pacote binário, qualquer outro byte é JSON.
O SOF de dois bytes deixa o parser se realinhar no meio das linhas de log.

### Diagnóstico no serial

No boot em I²C o firmware varre o barramento e lista quem responde, marcando
os conhecidos. Se o BMI323 não aparecer aí, o problema é elétrico
(alimentação, pull-up, fio) e não de configuração.

A cada 5 s sai uma linha de status:

```
[st] 1600/1589 Hz  rms 3.2->1.4 mg  |a| 0.994 g (98%)  roll 4.7 pitch -0.4 yaw -0.1
```

taxa configurada/real, RMS antes e depois dos filtros, módulo da gravidade com
a confiança do estimador, e a atitude. Comandos aceitos no serial:
`odr <Hz>`, `range <g>`, `bw <0|1>`, `gyro on|off`, `peaks`, `help`.

> Monte o sensor **rígido** na peça a medir. Fita dupla-face ou fio solto
> introduzem ressonâncias próprias e mascaram a vibração real.

## Compilar e gravar

```bash
pio run -t upload      # firmware
pio run -t uploadfs    # interface web (pasta data/)
pio device monitor     # log serial, 115200
```

Na primeira vez o ESP sobe um access point:

- SSID `BMI323-Vib` / senha `vibracao123` → abra `http://192.168.4.1/`
- Para entrar na sua rede: `http://192.168.4.1/api/wifi?ssid=SUAREDE&pass=SUASENHA`
  (o ESP reinicia e passa a responder também em `http://bmi323.local/`)

## Interface

| Painel | O que mostra |
|--------|--------------|
| **Leitura da IMU** | painel principal: o sinal **cru** dos 6 eixos no tempo, uma faixa por canal, cada uma com sua própria escala (é o que deixa mg e °/s conviverem). Vem de `copyScopeRaw()` — o buffer do FIFO convertido para g / °/s, sem passar por filtro nenhum. Os chips da legenda ligam e desligam cada canal, e canal desligado não gasta FFT nem banda de rede. O espectro continua sendo calculado: alimenta o waterfall e a tabela de picos. |
| **Waterfall** | histórico do espectro — mostra se a frequência de oscilação deriva com o tempo (ex.: rotação variando). |
| **Forma de onda** | sinal no tempo, cru e filtrado sobrepostos. |
| **Filtros** | diagrama da cadeia com os estágios ativos acesos, e um bloco por filtro com todos os parâmetros. Notch dinâmico e RPM mostram ao vivo as frequências que estão rastreando. |
| **Leitura dos eixos** | faixa acima do espectro: a leitura direta do chip, sem estatística no meio. Para cada um dos 6 eixos, a **contagem de 16 bits do último quadro do FIFO** — o valor que veio pelo barramento — e logo abaixo o mesmo número convertido pelo fundo de escala em vigor (g ou °/s). Não passa por filtro, média nem FFT: valor estranho aqui é problema de sensor ou de barramento, não de processamento. Atualiza junto com o status, 1×/s. |
| **Onda filtrada** | uma faixa por canal, só o sinal depois dos filtros, cada faixa com sua própria escala (mg e °/s convivem sem uma esmagar a outra). |
| **Atitude** | horizonte artificial (roll/pitch), bússola de yaw, taxas do giroscópio e a **confiança na gravidade**. |
| **Picos detectados** | as raias dominantes, com frequência refinada por interpolação parabólica. `+ filtrar` joga o pico na lista de **oscilações** do painel de filtros, que decide sozinha onde ele cai. |
| **Simulador** | gera vibração sintética no lugar do sensor: até 6 fontes rotativas, ressonância estrutural e movimento do corpo. |

Métricas no topo: taxa configurada e taxa real medida, RMS AC antes/depois do
filtro (o par mais útil para julgar se o filtro resolveu), pico a pico e
temperatura do sensor.

## Fluxo de trabalho típico

1. Escolha o eixo (ou **módulo**) e o ODR. Para vibração mecânica geral,
   1.6 kHz e FFT 1024 dão 1.56 Hz por bin até 800 Hz.
2. Deixe rodar com **suavização média** até o espectro estabilizar; use
   **max hold** se a vibração for intermitente.
3. Diga o que incomoda na caixa **OSCILAÇÕES**: digite `118,4`, ou várias de
   uma vez (`118 236 354`), ou clique em `+ filtrar` na tabela de picos. Cada
   chip vira um alvo e o painel monta a cadeia sozinho.
4. Ajuste o chip se precisar: clique no `Q` para trocar a largura
   (estreita/média/larga) e no `~` para dizer que a frequência varia — aí o
   alvo passa para o notch dinâmico em vez de um notch fixo.
5. Compare o RMS filtrado com o cru e a curva âmbar com a azul. A linha
   **corte · atraso · notches** logo abaixo do diagrama diz o preço: filtrar
   custa atraso de grupo, em ms.
6. **Salvar na flash** guarda tudo em NVS (volta assim no próximo boot).
7. **Código C dos filtros** gera os biquads prontos para colar no firmware do
   equipamento que você está tratando.
8. **Baixar CSV** exporta o buffer circular inteiro (cru + filtrado, 3 eixos)
   para análise externa.

## Simulador

Liga pela pílula **simulador** na barra de status superior e substitui o FIFO
do sensor. Desligado, o painel de configuração do simulador some da barra
lateral — não há o que ajustar, e a coluna se reorganiza sozinha. As
amostras sintéticas entram no **mesmo caminho** das amostras do BMI323 — FFT,
cadeia de filtros, notch dinâmico e estimativa de atitude são os reais. Serve
para projetar filtros sem a máquina na bancada e para exercitar o firmware
inteiro sem hardware nenhum ligado.

Cada uma das **6 fontes** tem:

| Parâmetro | O que faz |
|-----------|-----------|
| freq / rpm | fundamental (os dois campos são a mesma grandeza) |
| amp | amplitude da fundamental, em mg |
| harm + queda | nº de harmônicas e o quanto cada uma decai (`amp × queda^(n-1)`) |
| drift + período | excursão senoidal da rotação — é o que dá trabalho ao notch dinâmico |
| jitter | erro aleatório de rotação (passeio lento) |
| eixo | X, Y, Z ou todos (com pesos diferentes por eixo, como numa máquina real) |

Mais: **ruído** de fundo, **gravidade** (1 g em Z, para o DC-block ter o que
remover), **ressonância** da fixação (passa-faixa que realça uma banda, como a
estrutura onde a placa está presa faz de verdade) e **movimento** do corpo
(balanço em roll/pitch e giro em yaw), que alimenta a janela de atitude.

Presets prontos: drone 5" e 7" (4 motores com rotações ligeiramente diferentes),
motor elétrico de 4 polos em 60 Hz (1770 rpm + 120 Hz magnético), ventilador
axial de 5 pás (rotação + passagem de pá), bomba/compressor (harmônicas de folga
mecânica), desbalanceamento simples e **varredura** — um motor acelerando de
100 a 500 Hz, que é onde o notch dinâmico mostra serviço.

Sem firmware por trás (abrindo `data/index.html` direto), a página entra em modo
**OFFLINE** e a mesma configuração alimenta uma prévia analítica gerada no
navegador. Serve para desenhar filtros e conferir a interface; os números não
vêm do DSP real.

## Atitude — onde está a terra, mesmo com ruído

Este é o segundo objetivo do projeto e tem um problema próprio: sob vibração o
acelerômetro mede **gravidade + a vibração inteira**. Jogar isso num `atan2` faz
o ângulo pular junto com a vibração; simplesmente ignorar o acelerômetro quando
`|a|` foge de 1 g deixa o giroscópio sem correção e a deriva come o ângulo.

O estimador em `src/attitude.cpp` ataca isso em três frentes:

1. **Passa-baixa pesado (PT2, 3 Hz por padrão) só para estimar a gravidade.**
   Vibração mecânica vive acima de dezenas de Hz; gravidade é DC. Separar as
   duas no domínio da frequência resolve a maior parte antes de qualquer
   lógica — em 3 Hz, uma vibração de 100 Hz chega atenuada ~60 dB.
2. **Confiança gradual** em vez de liga/desliga: quanto mais o `|a|` filtrado se
   afasta de 1 g, menos peso o acelerômetro tem. Sem degraus, sem chaveamento
   nervoso. A interface mostra essa confiança em tempo real.
3. **Correção PI (estilo Mahony):** o termo integral estima o offset do
   giroscópio continuamente. É o que segura o ângulo nos trechos em que o
   acelerômetro não merece confiança — o giro já está compensado de antes, então
   sozinho aguenta bem mais tempo.

Ajustáveis no painel Aquisição: **LPF gravidade** (corte do PT2) e
**Resposta (kp)** (ganho do complementar; `1/kp` é a constante de tempo).

### Fonte do sinal

O campo **Fonte da atitude** escolhe o que alimenta o estimador:

| Opção | Acelerômetro | Giroscópio |
|-------|--------------|------------|
| cru | cru | cru |
| **giro filtrado** (padrão) | cru | **filtrado pela cadeia** |
| tudo filtrado | filtrado¹ | filtrado |

Filtrar o giroscópio **antes de integrar** é ganho direto: os notches e o
passa-baixa tiram a vibração que iria virar erro de ângulo.

¹ O acelerômetro só pode vir filtrado com o **DC-block desligado** — com ele
ligado não sobra gravidade para referenciar nada. Se você escolher "tudo
filtrado" com o DC-block ativo, o firmware volta ao acelerômetro cru sozinho e a
interface avisa na linha de status. Isso não é perda: o próprio estimador já tem
o PT2 de 3 Hz, que isola a gravidade melhor do que a cadeia de vibração faria.

### Yaw deriva

`roll` e `pitch` têm referência absoluta e sobrevivem à vibração. **`yaw` não**:
é integração pura de `gz` e, sem magnetômetro, não há como corrigir. Serve para
rotação relativa, não para rumo — o instrumento está rotulado "YAW (deriva)".

O offset do giroscópio é medido em ~1 s no boot; **calibrar** refaz a medida
(deixe parado) e **zerar yaw** zera a referência. Desligar o giroscópio no painel
Aquisição volta o FIFO para 3 words por quadro; roll e pitch continuam saindo do
acelerômetro filtrado e yaw fica indisponível.

## Cadeia de filtros — porte do Betaflight

Os filtros são os do Betaflight, portados de `src/main/common/filter.c`,
`common/sdft.c`, `sensors/gyro_filter_impl.c` e `flight/dyn_notch_filter.c`.
A ordem é a mesma do `gyro_filter_impl.c`:

```
[DC-block] → LPF2 → RPM → notch1 → notch2 → LPF1 → notch dinâmico (SDFT)
```

| Bloco | Parâmetros | Equivalente no Betaflight |
|-------|-----------|---------------------------|
| **Oscilações** | lista de frequências-alvo + largura; deriva RPM, notch 1/2 e notch dinâmico | *não existe* — ver abaixo |
| **DC-block** | on/off, corte | *não existe* — ver abaixo |
| **Multiplicador global** | 25–200 % sobre todos os cortes de LPF | `simplified_gyro_filter_multiplier` |
| **LPF2** | tipo PT1/BIQUAD/PT2/PT3, corte | `gyro_lpf2_type`, `gyro_lpf2_static_hz` |
| **RPM** | fonte, base, harmônicas 0–3, Q, mín, fade, suavização da fundamental, peso por harmônica | `rpm_filter_*` |
| **Notch 1 / 2** | centro + corte inferior (o Q sai de `filterGetNotchQ`) | `gyro_notch1/2_hz`, `gyro_notch1/2_cutoff` |
| **LPF1** | tipo, corte estático **ou** faixa dinâmica (mín/máx/expo) | `gyro_lpf1_type`, `gyro_lpf1_static_hz`, `gyro_lpf1_dyn_min_hz/max_hz/expo` |
| **Notch dinâmico** | quantidade 0–3, Q, mín, máx | `dyn_notch_count/q/min_hz/max_hz` |

Filtro de hardware do sensor (`gyro_hardware_lpf`) fica no painel **Aquisição**:
largura de banda ODR/2 ou ODR/4 e média por hardware.

### Guia de ajuste

`guia-filtros.html`, na raiz do repositório, documenta os sete estágios um por
um: o que cada um faz no sinal, quando vale mexer, o que digitar, e quanto de
atraso custa. As curvas de resposta são calculadas em tempo de carga com as
mesmas fórmulas de `bf_filter.cpp`, a 1600 Hz — não são figuras. Tem também um
laboratório interativo que mostra a curva da cadeia e o atraso de grupo
mudando juntos.

Abra o arquivo no navegador, ou pelo endereço publicado:
[engenharia10.github.io/bmi323-vibration-analyzer/guia-filtros.html](https://engenharia10.github.io/bmi323-vibration-analyzer/guia-filtros.html)
— o workflow do Pages copia a raiz para o site, porque o GitHub não renderiza
HTML no navegador de arquivos. O link **guia** no cabeçalho do painel de
filtros aponta para esse endereço, e por isso precisa de internet: o arquivo
não está mais no LittleFS da placa.

Todo valor ao lado de um slider aceita clique para digitar (Enter confirma,
Esc cancela) — é o único jeito de cravar os 118,4 Hz que o analisador mediu.

### Oscilações: a entrada de cima

O bloco **OSCILAÇÕES** não tem equivalente no Betaflight, e por um motivo
específico: lá a frequência do motor chega pronta pela telemetria DShot dos
ESCs, então o piloto nunca precisa digitá-la. Aqui a fonte é o usuário e o
próprio espectro. A lista é a entrada natural, e um alocador decide o destino
de cada alvo:

- **família harmônica** — dois ou três alvos em 2× e 3× do menor (±3%) viram
  *um* filtro RPM: base na fundamental, `rpmHarm` = quantos;
- **alvo fixo isolado** — vai para `notch1`, depois `notch2`. O centro é a
  frequência digitada e o corte sai do Q, invertendo `filterGetNotchQ`:
  `fc = f₀·(√(1+4Q²)−1)/(2Q)` — 0,905·f₀ para Q 5, 0,847·f₀ para Q 3,
  0,721·f₀ para Q 1,5;
- **alvo que varia** (`~`) — alimenta o notch dinâmico, com `dnMin` em 0,7× o
  menor e `dnMax` em 1,4× o maior;
- **estouro** — mais de dois alvos fixos sem relação harmônica não cabem em
  dois notches; o painel avisa e sugere marcar um como variável.

A lista escreve na config só quando muda. Os campos de `centro`, `corte` e `Q`
continuam editáveis no painel — o que garante que o porte siga auditável — e
na primeira carga a lista é reconstruída a partir da config que está na placa.

### Três adaptações, e por quê

1. **DC-block** — o Betaflight filtra o **giroscópio**, que não tem componente
   contínua. Aqui o sinal é aceleração: sem esse passa-alta a gravidade (1 g)
   domina o espectro inteiro. É o único bloco que não vem do Betaflight, e está
   marcado como tal na interface.
2. **Filtro RPM** — no Betaflight a frequência vem da telemetria bidirecional
   DShot dos ESCs. Aqui você escolhe a fonte: **manual** (digita a fundamental
   em Hz) ou **auto**, que usa o pico dominante da FFT. A lógica de peso e de
   fade perto de `min_hz` é a do original.
3. **LPF1 dinâmico** — no Betaflight o corte varia com o acelerador. Aqui a
   mesma curva (`dynLpfCutoffFreq`) é dirigida por um slider **carga** 0–100 %,
   para você ver o efeito sem ter um acelerador.

O filtro de RPM por telemetria de ESC e a compensação de fase não foram
portados por dependerem de hardware que não existe aqui.

### Sobre o BMI323

Betaflight suporta MPU6000/6050/6500/9250, ICM-4xxxx/ICM-2xxxx, BMI160,
**BMI270** e LSM6DSO — o **BMI323 não está na lista**. O parente mais próximo é
o BMI270, mas o mapa de registradores é outro, então o driver em `lib/BMI323`
foi escrito do zero a partir do datasheet. A cadeia de filtros, por operar
sobre `float`, é independente do sensor e portou 1:1.

### Precisão da curva na tela

As fórmulas RBJ e os ganhos PT1/PT2/PT3 estão duplicados em `data/app.js`
com as mesmas constantes de correção de corte (`1.553773974`, `1.961459177`).
A curva rosa `|H(f)|` é a resposta real da cadeia rodando no ESP32, incluindo
os notches dinâmicos e de RPM nas posições que o firmware está rastreando
naquele instante — não é aproximação.

## Estrutura

```
platformio.ini          ambiente, flags e dependências
partitions_custom.csv   2 MB app + ~2 MB LittleFS
include/config.h        pinos, ODR padrão, tamanhos de buffer

lib/BMI323/             driver do sensor (SPI/I2C runtime, FIFO) — sem
  src/bmi323.{h,cpp}    dependência do resto do projeto

lib/Filtros/            porte dos filtros do Betaflight (GPLv3)
  src/bf_filter.{h,cpp} PT1/PT2/PT3, biquad DF1/DF2, filterGetNotchQ
  src/bf_sdft.{h,cpp}   SDFT deslizante de 72 amostras / 36 bins
  src/filtros.{h,cpp}   cadeia completa + notch dinâmico + filtro RPM

lib/Simulador/          gerador de vibração sintética
  src/simulador.{h,cpp} fontes multi-harmônicas, ressonância, movimento

src/attitude.{h,cpp}    complementar PI: pitch, roll e yaw sob vibração
src/dsp.{h,cpp}         FFT radix-2, janelas, detecção de picos, RMS
src/analyzer.{h,cpp}    tasks de aquisição e DSP, buffer circular, CSV
src/webui.{h,cpp}       HTTP + WebSocket + API REST
src/settings.{h,cpp}    persistência em NVS
src/main.cpp            inicialização
data/                   interface web (index.html, style.css, app.js)
```

Aquisição roda no core 0 (prioridade 6), DSP no core 1, servidor web no loop
principal. Buffer circular de 2048 amostras × 3 eixos, cru e filtrado.

A máquina de estados do notch dinâmico roda uma vez por amostra, igual ao
Betaflight: 4 passos × 3 eixos = 12 iterações para atualizar tudo. A única
mudança é o piso de taxa — o Betaflight exige 2 kHz de loop, aqui `500 Hz`
(`DYN_NOTCH_MIN_SAMPLE_RATE`), porque a dinâmica de uma máquina é muito mais
lenta que a de um drone.

## API HTTP

Todas respondem com o JSON de configuração atual.

| Rota | Parâmetros |
|------|-----------|
| `GET /api/config` | — |
| `GET /api/sensor` | `odr` (Hz), `range` (2/4/8/16), `avg` (0..6), `bw` (0/1), `gyro` (0/1), `gyrRange` (125..2000) |
| `GET /api/analysis` | `fft`, `window` (0..4), `alpha`, `axis` (0..3) |
| `GET /api/filters` | `dc`, `dcf`, `mult`, `lpf1Type`, `lpf1Hz`, `lpf1Dyn`, `lpf1DynMin`, `lpf1DynMax`, `lpf1DynExpo`, `lpf2Type`, `lpf2Hz`, `n1h`, `n1c`, `n2h`, `n2c`, `dnCount`, `dnQ`, `dnMin`, `dnMax`, `rpmSrc`, `rpmBase`, `rpmHarm`, `rpmQ`, `rpmMin`, `rpmFade`, `rpmLpf`, `rpmW0..2`, `load` |
| `GET /api/sim` | `simOn`, `simNoise`, `simGrav`, `simResOn`, `simResHz`, `simResQ`, `simResG`, `simAtt`, `simRoll`, `simPitch`, `simAttP`, `simYaw`, e por fonte `s{0..5}` + `en/f/a/h/d/dr/dp/jt/ax` |
| `GET /api/attitude` | `cal=1` recalibra o giroscópio, `zero=1` zera o yaw |
| `GET /api/save` | grava a configuração atual na NVS |
| `GET /api/defaults` | volta aos padrões |
| `GET /api/csv` | download do buffer circular |
| `GET /api/wifi` | `ssid`, `pass` (reinicia) |

WebSocket em `/ws`: pacotes binários (cabeçalho de 16 bytes + `float32[]`,
tipo 1 = espectro, tipo 2 = forma de onda) e JSON de status a 5 Hz.

## Limites conhecidos

- **Yaw deriva** — integração pura do giroscópio, sem magnetômetro para corrigir.
- O simulador não modela fase entre eixos nem acoplamento estrutural real; ele
  soma senoides e passa por uma ressonância. Serve para projetar filtro, não
  para substituir medição.
- O filtro RPM não usa telemetria de ESC (não existe aqui): a fundamental é
  manual ou vem do pico dominante da FFT.
- O LPF1 dinâmico é dirigido pelo slider **carga**, não pelo acelerador.
- Espectro e forma de onda são transmitidos **apenas do eixo selecionado** — os
  três eixos são sempre adquiridos e filtrados, mas só um é analisado por vez
  (mantém a banda de rede e a CPU folgadas).
- FFT máxima 1024 pontos (`FFT_MAX` em `config.h`); subir exige mais RAM.
- O giroscópio fica desligado: o foco é vibração linear.

## Licença

`lib/Filtros` é um porte de código do Betaflight e mantém a licença original,
**GPL-3.0-or-later**. O restante do projeto segue a mesma licença por
consequência do link estático.

## Memória

Os buffers grandes (anel dos 6 canais, espectros médios, buffer de transmissão)
vão para a **PSRAM** quando ela existe — o padrão de acesso é sequencial, que é
onde a PSRAM se sai bem. O scratch da FFT fica de propósito na RAM interna: ali
o acesso é aleatório e quente. Com PSRAM, a RAM interna fica em ~32 %.

`platformio.ini` assume PSRAM **octal** (`qio_opi`), o caso das devkits S3
N8R8 / N16R8. Se a sua for de PSRAM quad (N8R2), troque para `qio_qspi`. Errar
aqui não impede de rodar: sem PSRAM o firmware cai de volta na RAM interna
sozinho e avisa no serial.

O sinal cru é guardado em **contagens nativas do sensor** (int16) em vez de
float: ocupa metade da memória e não perde nada, porque é exatamente a resolução
que o BMI323 entrega.
