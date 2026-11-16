# Monitoramento de Gás e Fumaça com Exaustão Automática

Projeto da disciplina **Sistemas de Tempo Real** (Ciência da Computação, IESB), desenvolvido por Danielle Lima da Silva, Davi Marques de Souza e Lucca di Stefanno Brito Serafim.

O firmware roda em uma **ESP32** com **FreeRTOS** (ESP-IDF 6.1.0) e foi testado no simulador **Wokwi**. Um sensor MQ-2 classifica o ambiente em *normal*, *alerta* ou *crítico* e aciona LEDs, buzzer e um servo motor que representa a abertura de um mecanismo de exaustão. A aplicação também lê temperatura e umidade (DHT22), permite silenciar o buzzer por botão e inclui uma tarefa de carga computacional, usada para observar o comportamento das demais tarefas sob processamento adicional.

## Estados e respostas

Os limites são valores do ADC, definidos em `main/main.c`:

| Estado  | Leitura do MQ-2 | LED      | Servo | Buzzer                      |
|---------|-----------------|----------|-------|-----------------------------|
| Normal  | abaixo de 3000  | verde    | 0°    | desligado                   |
| Alerta  | 3000 a 3999     | amarelo  | 45°   | ligado, salvo silenciamento |
| Crítico | 4000 ou mais    | vermelho | 90°   | ligado, salvo silenciamento |

Esses limites são parâmetros da simulação e **não** representam uma concentração de gás calibrada em ppm.

## Pinagem

| Componente              | GPIO         |
|-------------------------|--------------|
| MQ-2 (saída analógica)  | 34 (ADC1 canal 6) |
| DHT22 (dados)           | 4            |
| Botão                   | 33           |
| LEDs verde/amarelo/vermelho | 25 / 26 / 27 |
| Buzzer                  | 14           |
| Servo                   | 13           |

## Circuito (`diagram.json`)

- MQ-2 alimentado em 5 V, com a saída analógica ligada diretamente ao GPIO 34.
- DHT22 em 3V3.
- LEDs com resistores de 220 Ω.
- Servo em 5 V, com GND comum.
- Botão entre o GPIO 33 e o GND, usando o pull-up interno (pressionado = nível baixo).

> **Atenção:** a ligação direta do MQ-2 ao GPIO 34 vale apenas para a simulação. Em uma montagem física, a saída analógica precisa ser condicionada (por exemplo, com divisor resistivo) para não exceder a tensão segura do GPIO da ESP32.

## Arquitetura

O fluxo principal de segurança é T1 → T2 → T3. As demais tarefas acrescentam funcionalidades sem substituir esse fluxo.

| Tarefa | Função | Prioridade | Ativação |
|--------|--------|-----------|----------|
| T1 | Lê o MQ-2 (média de 8 amostras) e envia à fila | 2 | Periódica, 100 ms (`vTaskDelayUntil`) |
| T2 | Classifica o risco e gera o comando | 3 | Bloqueada na fila de leituras |
| T3 | Atualiza LEDs, servo e buzzer | 3 | Bloqueada em notificação de tarefa |
| T4 | Carga computacional (10.000 iterações de `sqrt`, `sin` e `cos` por ciclo) | 1 | Contínua, com espera de 10 ms entre ciclos |
| T5 | Lê temperatura e umidade do DHT22 | 1 | Periódica, 2 s |
| T6 | Detecta o botão e alterna o silenciamento (debounce de 35 ms) | 2 | Varredura a cada 10 ms |

**Comunicação e sincronização**

- T1 → T2: fila FreeRTOS com 10 posições, transportando o valor lido e o instante da aquisição. O envio é feito sem espera; se a fila estiver cheia, a amostra é descartada e contada.
- T2 → T3: fila de comandos (10 posições) mais notificação direta. A fila transporta os dados e a notificação sinaliza o evento.
- Leitura do DHT22: seção crítica para preservar os intervalos do protocolo.
- Os atuadores são controlados pela T3. A exceção é o buzzer, que a T6 também atualiza ao alternar o silenciamento.

## Requisitos temporais e medição

O firmware mede e imprime, a cada 10 s, um relatório de estatísticas pelo monitor serial.

| ID | Evento | Limite |
|----|--------|--------|
| RT01 | Intervalo entre aquisições consecutivas do MQ-2 | 100 ms |
| RT02 | Da marca temporal da leitura até o fim da classificação (T2) | 50 ms |
| RT03 | Da marca temporal da leitura até o fim da atuação (T3), ponta a ponta | 100 ms |

## Configuração

- `HABILITAR_T4` (em `main/main.c`): `1` liga a tarefa de carga e `0` a desliga.
- `CONFIG_FREERTOS_HZ=1000` (em `sdkconfig.defaults`): tick do FreeRTOS de 1 ms.
- Alvo: `esp32`.

## Compilar

No terminal do ESP-IDF, dentro da pasta do projeto:

```bash
idf.py set-target esp32
idf.py build
```

## Executar no Wokwi

1. Abra a pasta no VS Code com as extensões **ESP-IDF** e **Wokwi Simulator** instaladas e com a licença do Wokwi ativada.
2. Compile o projeto (**ESP-IDF: Build Project**). A simulação usa o firmware já compilado; sempre recompile depois de alterar o código.
3. Pressione F1 e escolha **Wokwi: Start Simulator**.
4. Clique no sensor de gás no diagrama para mexer no controle deslizante e passar pelos três estados. Clique no botão azul para silenciar ou reativar o buzzer.

A configuração do simulador está no `wokwi.toml` da raiz.

## Resultados observados (simulação)

Foram feitas duas execuções, com durações diferentes:

| Métrica | A: T4 desligada (~180 s) | B: T4 ligada (~30 s) |
|---------|--------------------------|----------------------|
| Leituras do MQ-2 | 1802 | 303 |
| RT01 (intervalo máx.) | 102,842 ms, 8 violações | 101,881 ms, 1 violação |
| RT02 (latência máx.) | 45 µs, 0 violações | 45 µs, 0 violações |
| RT03 (latência máx.) | 3,271 ms, 0 violações | 3,728 ms, 0 violações |
| Perdas nas filas | 0 | 0 |
| DHT22 | 90 leituras, 0 erros | 16 leituras, 0 erros |

Na execução B, a T4 completou 6 ciclos de 10.000 operações, com cerca de 4,46 s cada, e as leituras do MQ-2 continuaram sendo registradas a cada 100 ms durante esse processamento.

## Limitações

- Os testes foram feitos apenas no Wokwi, sem validação em componentes físicos.
- Os limites do MQ-2 são valores de ADC da simulação, sem calibração.
- O RT01 foi violado por até cerca de 2,8 ms, inclusive com a T4 desligada. A causa ainda não foi identificada.
- As duas execuções têm durações diferentes, então a comparação entre elas não comprova a ausência de efeito da T4.
- O RT03 inclui a impressão no monitor serial feita pela T3.
- A leitura do DHT22 usa temporização por software e seção crítica; o comportamento deve ser reavaliado em hardware físico.
- Os resultados são evidências dessas execuções específicas, não uma garantia geral de cumprimento de prazos.

## Estrutura do repositório

```
.
├── CMakeLists.txt        # projeto ESP-IDF (Monitoramento_Gas)
├── diagram.json          # circuito do Wokwi
├── wokwi.toml            # configuração do simulador
├── sdkconfig.defaults    # alvo esp32 e tick de 1000 Hz
└── main/
    ├── CMakeLists.txt
    └── main.c            # firmware
```