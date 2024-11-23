import neopixel
from machine import Pin, Timer
import time

ws_pin = 0
led_num = 109

neoRing = neopixel.NeoPixel(Pin(ws_pin), led_num)

bouton1 = Pin(3, machine.Pin.IN, pull=Pin.PULL_DOWN)
bouton2 = Pin(5, machine.Pin.IN, pull=Pin.PULL_DOWN)
bouton3 = Pin(11, machine.Pin.IN, pull=Pin.PULL_DOWN)
bouton4 = Pin(9, machine.Pin.IN, pull=Pin.PULL_DOWN)

def set_brightness(color, brightness):
    r, g, b = color
    r = int(r * brightness)
    g = int(g * brightness)
    b = int(b * brightness)
    return (r, g, b)

none_color = (0, 0, 0)  # Black
none_color = set_brightness(none_color, 0)
    
white_color = (255, 255, 255)  # Black
white_color = set_brightness(white_color, 0.1)
    
red_color = (255, 0, 0)  # Red color
red_color = set_brightness(red_color, 0.1)
    
blue_color = (0, 0, 255)  # blue color
blue_color = set_brightness(blue_color, 0.1)
    
green_color = (0, 255, 0)  # blue color
green_color = set_brightness(green_color, 0.1)
    
yellow_color = (255, 255, 0)  # blue color
yellow_color = set_brightness(yellow_color, 0.1)

player1_position = 0
player1_button_state = False

player2_position = 0
player2_button_state = False

player3_position = 0
player3_button_state = False

player4_position = 0
player4_button_state = False



red_player_active = False
blue_player_active = False
yellow_player_active = False
green_player_active = False

winner = 0


tim = Timer()
countdown = 7

is_running = False

bouton_rouge_led = Pin(4, machine.Pin.OUT)
bouton_rouge_led.off()

bouton_green_led = Pin(10, machine.Pin.OUT)
bouton_green_led.off()

bouton_yellow_led = Pin(12, machine.Pin.OUT)
bouton_yellow_led.off()

bouton_bleu_led = Pin(6, machine.Pin.OUT)
bouton_bleu_led.off()

def loop():
    neoRing.fill(none_color)
    
    if winner == 0:
        if is_running:
            run_refresh()
        else:
            menu()
    else:
        if winner == 1:
            neoRing.fill(red_color)
            
        if winner == 2:
            neoRing.fill(blue_color)
            
        if winner == 3:
            neoRing.fill(yellow_color)
            
        if winner == 4:
            neoRing.fill(green_color)
            
        neoRing.write()

def menu():
    global red_player_active
    global blue_player_active
    global yellow_player_active
    global green_player_active
    
    def num_players():
        return (red_player_active == True) + (blue_player_active == True) + (yellow_player_active == True) + (green_player_active == True)
    
    def add_time(self):
        global countdown
        global is_running
        
        countdown = countdown - 1
        
        if countdown == 0:
            tim.deinit()
            is_running = True
    
    if red_player_active == False and bouton1.value() == 1:
        red_player_active = True
        bouton_rouge_led.on()
        
        if(num_players() == 2):
            tim.init(period=1000, mode=Timer.PERIODIC, callback=add_time)

            
    
    if blue_player_active == False and bouton2.value() == 1:
        blue_player_active = True
        bouton_bleu_led.on()
        
        if(num_players() == 2):
            tim.init(period=1000, mode=Timer.PERIODIC, callback=add_time)
        
    if yellow_player_active == False and bouton3.value() == 1:
        yellow_player_active = True
        bouton_yellow_led.on()
        
        if(num_players() == 2):
            tim.init(period=1000, mode=Timer.PERIODIC, callback=add_time)
        
    if green_player_active == False and bouton4.value() == 1:
        green_player_active = True
        bouton_green_led.on()
        
        if(num_players() == 2):
            tim.init(period=1000, mode=Timer.PERIODIC, callback=add_time)
    
    if yellow_player_active == True :
        neoRing[1] = yellow_color
        neoRing[2] = yellow_color
        neoRing[3] = yellow_color
        neoRing[4] = yellow_color
        neoRing[5] = yellow_color
        neoRing[6] = yellow_color
        
        neoRing[29] = yellow_color
        neoRing[30] = yellow_color
        neoRing[31] = yellow_color
        neoRing[32] = yellow_color
        neoRing[33] = yellow_color
        
        neoRing[53] = yellow_color
        neoRing[54] = yellow_color
        neoRing[55] = yellow_color
        neoRing[56] = yellow_color
        
        neoRing[73] = yellow_color
        neoRing[74] = yellow_color
        neoRing[75] = yellow_color
        
        neoRing[89] = yellow_color
        neoRing[90] = yellow_color
        
        neoRing[101] = yellow_color
    
    if blue_player_active == True :
        neoRing[8] = blue_color
        neoRing[9] = blue_color
        neoRing[10] = blue_color
        neoRing[11] = blue_color
        neoRing[12] = blue_color
        neoRing[13] = blue_color
        
        neoRing[35] = blue_color
        neoRing[36] = blue_color
        neoRing[37] = blue_color
        neoRing[38] = blue_color
        neoRing[39] = blue_color
        
        neoRing[58] = blue_color
        neoRing[59] = blue_color
        neoRing[60] = blue_color
        neoRing[61] = blue_color
        
        neoRing[77] = blue_color
        neoRing[78] = blue_color
        neoRing[79] = blue_color
        
        neoRing[92] = blue_color
        neoRing[93] = blue_color
        
        neoRing[103] = blue_color

    if red_player_active == True :
        neoRing[15] = red_color
        neoRing[16] = red_color
        neoRing[17] = red_color
        neoRing[18] = red_color
        neoRing[19] = red_color
        neoRing[20] = red_color
        
        neoRing[41] = red_color
        neoRing[42] = red_color
        neoRing[43] = red_color
        neoRing[44] = red_color
        neoRing[45] = red_color
        
        neoRing[63] = red_color
        neoRing[64] = red_color
        neoRing[65] = red_color
        neoRing[66] = red_color
        
        neoRing[81] = red_color
        neoRing[82] = red_color
        neoRing[83] = red_color
        
        neoRing[95] = red_color
        neoRing[96] = red_color
        
        neoRing[105] = red_color
    
    if green_player_active == True :
        neoRing[22] = green_color
        neoRing[23] = green_color
        neoRing[24] = green_color
        neoRing[25] = green_color
        neoRing[26] = green_color
        neoRing[27] = green_color
        
        neoRing[47] = green_color
        neoRing[48] = green_color
        neoRing[49] = green_color
        neoRing[50] = green_color
        neoRing[51] = green_color
        
        neoRing[68] = green_color
        neoRing[69] = green_color
        neoRing[70] = green_color
        neoRing[71] = green_color
        
        neoRing[85] = green_color
        neoRing[86] = green_color
        neoRing[87] = green_color
        
        neoRing[98] = green_color
        neoRing[99] = green_color
        
        neoRing[107] = green_color
    
    if num_players() > 1:
        neoRing[108] = white_color
        
        if countdown >= 2:
            neoRing[106] = white_color
            neoRing[104] = white_color
            neoRing[102] = white_color
            neoRing[100] = white_color
            
        if countdown >= 3:
            neoRing[97] = white_color
            neoRing[94] = white_color
            neoRing[91] = white_color
            neoRing[88] = white_color
            
        if countdown >= 4:
            neoRing[84] = white_color
            neoRing[80] = white_color
            neoRing[76] = white_color
            neoRing[72] = white_color

        if countdown >= 5:
            neoRing[52] = white_color
            neoRing[57] = white_color
            neoRing[62] = white_color
            neoRing[67] = white_color

        if countdown >= 6:
            neoRing[46] = white_color
            neoRing[40] = white_color
            neoRing[34] = white_color
            neoRing[28] = white_color

        if countdown >= 7:
            neoRing[21] = white_color
            neoRing[14] = white_color
            neoRing[7] = white_color
            neoRing[0] = white_color

    
    neoRing.write()

def end_game(self):
    global player1_position
    global player2_position
    global player3_position
    global player4_position

    global player1_button_state
    global player2_button_state
    global player3_button_state
    global player4_button_state
    
    global winner
    global is_running
    
    global green_player_active
    global red_player_active
    global yellow_player_active
    global blue_player_active
    global countdown
    
    player1_position = 0
    player2_position = 0
    player3_position = 0
    player4_position = 0
    
    player1_button_state = False
    player2_button_state = False
    player3_button_state = False
    player4_button_state = False
    
    green_player_active = False
    red_player_active = False
    yellow_player_active = False
    blue_player_active = False
    
    countdown = 7
    
    winner = 0
    is_running = False
    
    bouton_rouge_led.off()
    bouton_green_led.off()
    bouton_yellow_led.off()
    bouton_bleu_led.off()

def run_refresh():
    global red_color
    global blue_color
    global yellow_color
    global green_color
    
    global player1_position
    global player2_position
    global player3_position
    global player4_position

    global player1_button_state
    global player2_button_state
    global player3_button_state
    global player4_button_state
    
    global winner
    
    #if player1_position == player2_position:
    #    neoRing[player1_position] = set_brightness((255, 0, 255), 1)
    
    
    if red_player_active:
        neoRing[player1_position] = red_color

        if player1_button_state == False and bouton1.value() == 1 and player1_position < 108:
            player1_button_state = True 
            player1_position = player1_position + 1
        
        if player1_button_state == True and bouton1.value() == 0:
            player1_button_state = False
            
            if player1_position == 108:
                winner = 1
                tim.init(period=5000, mode=Timer.ONE_SHOT, callback=end_game)


    if blue_player_active:
        neoRing[player2_position] = blue_color

        if player2_button_state == False and bouton2.value() == 1 and player2_position < 108:
            player2_button_state = True
            player2_position = player2_position + 1
        
        if player2_button_state == True and bouton2.value() == 0:
            player2_button_state = False
            
            if player2_position == 108:
                winner = 2
                tim.init(period=5000, mode=Timer.ONE_SHOT, callback=end_game)

    
    if yellow_player_active:
        neoRing[player3_position] = yellow_color

        if player3_button_state == False and bouton3.value() == 1 and player3_position < 108:
            player3_button_state = True
            player3_position = player3_position + 1
        
        if player3_button_state == True and bouton3.value() == 0:
            player3_button_state = False
            
            if player3_position == 108:
                winner = 3
                tim.init(period=5000, mode=Timer.ONE_SHOT, callback=end_game)


    if green_player_active:
        neoRing[player4_position] = green_color

        if player4_button_state == False and bouton4.value() == 1 and player4_position < 108:
            player4_button_state = True
            player4_position = player4_position + 1
        
        if player4_button_state == True and bouton4.value() == 0:
            player4_button_state = False

            if player4_position == 108:
                winner = 4
                tim.init(period=5000, mode=Timer.ONE_SHOT, callback=end_game)

    neoRing.write()

while True:
    loop()