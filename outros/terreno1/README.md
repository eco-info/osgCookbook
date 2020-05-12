# terreno1

Primeiro terreno controlado por mouse e teclado.

A tela pode variar entre tela-cheia e janela, comentando ou não a linha 211 (viewer.setUpViewInWindow).

![](terr1a.png)

![](terr1b.png)

## Prioridade 1

### Não trancar o mouse nas bordas da janela.

Tentativa:

- linha 197: centerMousePointer( ea, aa );
- junto com linhas 139 e 140

Problema: Perda significativa de desempenho. Dizem que há uma maneira de jogar o cursor do mouse para o centro da janela a cada frame, sem chamar as funções `handle` e `performMovement`, mas ainda não encontrei.

### Esconder o cursor.

Solução:

- linhas 216 e 217

Dúvida: não sei qual a diferença entre as duas. Aparentemente as duas fazem a mesma coisa.

### Usar as teclas não para mover diretamente, mas para acelerar e frear os motores do "OVNI".

Como está atualmente, uma tecla liberada (KEYUP) impede o movimento de outra que continua pressionada (KEYDOWN).

### Colocar a câmera inicial no alto, centralizada sobre o país, de forma que o mesmo caiba inteiro na tela (com o norte ou o sul pra cima, dependendo da configuração).

## Prioridade 2

- Acrescentar HUD (heads-up display)
- Usar teclas QE para girar ao redor do eixo vertical
- Usar teclas R/PageUp para ganhar altitude, e F/PageDown para perder altitude
- Usar teclas Shift para acelerar
- Usar tecla Tab (alguma outra?) para alternar mouse entre navegação (MODO A: girar câmera, cursor invisível) e movimento do cursor (MODO B: câmera parada, cursor visível)
- MODO B: identificar elementos da geografia, com nomes no HUD
- Mudar a posição do Sol segundo horário e estação do ano
- Inserir shapefiles
