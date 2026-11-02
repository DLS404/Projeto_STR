# Monitoramento de Gás — ESP-IDF 5.5.5

Projeto-base para ESP32 DevKit V1 no Wokwi.

## Pinagem
- MQ-2 AO: GPIO34 (com divisor 10k/20k)
- DHT22 DATA: GPIO4
- Botão: GPIO33 (pull-up interno; pressionado = LOW)
- LEDs verde/amarelo/vermelho: GPIO25/26/27
- Buzzer: GPIO14
- Servo: GPIO13

## Circuito
O diagrama usa:
- MQ-2 alimentado em 5V; saída AO passa por divisor resistivo 10k em série e 20k para GND.
- DHT22 em 3V3.
- LEDs com resistores de 220 ohms.
- Servo em 5V e GND comum.
- Botão entre GPIO33 e GND (pull-up interno).

## Compilar
No terminal ESP-IDF:
```bash
idf.py set-target esp32
idf.py build
```

## Executar no Wokwi
Abra a pasta no VS Code com as extensões ESP-IDF e Wokwi instaladas. Use o Wokwi Simulator para iniciar a simulação.

## Estado desta versão
A estrutura ESP-IDF, a pinagem e o diagrama do circuito estão preparados. O código inicial contém as tarefas FreeRTOS e os drivers GPIO, ADC e LEDC. A leitura efetiva do DHT22 ainda precisa receber um driver compatível com ESP-IDF 5.5.5; a tarefa está isolada para essa implementação na próxima etapa.
