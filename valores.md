### Sugestão Inicial para os Valores PID
Para um robô seguidor de linha, valores iniciais comuns para cada parâmetro são:

- *Kp*: Um valor inicial de *Kp* = 1.5 a 3.5 geralmente oferece um bom ponto de partida. Um *Kp* muito alto causará oscilações, enquanto um muito baixo fará com que o robô reaja lentamente.
- *Ki*: O valor de *Ki* deve ser pequeno, pois o termo integral é sensível ao acúmulo de erro ao longo do tempo. Valores entre 0.01 e 0.1 são recomendados para suavizar o ajuste sem causar windup.
- *Kd*: O valor de *Kd* ajuda a suavizar as oscilações causadas pelo *Kp*. Valores típicos ficam entre 0.1 e 1.0, mas variam dependendo da resposta desejada.


### Ajuste Fino e Testes
É importante testar esses valores no seu robô e ajustar conforme necessário:

- Aumente *Kp* se o robô está lento para corrigir o erro.
- Aumente *Ki* levemente se o robô tende a ficar afastado do centro por longos períodos, mas observe o windup.
- Ajuste *Kd* para reduzir oscilações, caso ocorram.

Esses valores são apenas um ponto de partida. Testar e ajustar será necessário para encontrar a melhor resposta para o seu robô em particular.


#### Código
```
float Kp = 2.0;      // Proporcional: controla a reação ao erro
float Ki = 0.05;     // Integral: suaviza ajustes pequenos ao longo do tempo
float Kd = 0.5;      // Derivativo: reduz oscilações e responde a mudanças rápidas
float P = 0, I = 0, D = 0, PID_value = 0, previous_error = 0, previous_I = 0;
```