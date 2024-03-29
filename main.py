#https://wokwi.com/projects/387024410728595457 
from machine import Pin, SPI,SoftI2C
import ili9342c

import random
import focaltouch
import math

i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
touch = focaltouch.FocalTouch(i2c)
def tftinit():
    
    spi = SPI(2,baudrate=60000000,sck=Pin(18),mosi=Pin(23)) # SPI init
    tft = ili9342c.ILI9342C(spi, cs=Pin(5, Pin.OUT), dc=Pin(15, Pin.OUT), rst=Pin(33, Pin.OUT),width=320, height=240,rotation=270)
    return tft
            
def main():
    #print("Wokwi-Simulator")
    tft = tftinit()
    tft.clear(ili9342c.WHITE)
    class Button:
        def __init__(self,x,y,breite,höhe,color,tft):
            self.x=x
            self.y=y
            self.breite=breite
            self.höhe=höhe
            self.color=color
            self.tft=tft
        
        def set_color(self,color):
            self.color = color
        def get_color(self):
            return self.color
        def draw(self):
            tft.fill_rect(self.x,self.y,self.breite,self.höhe,self.color)
        def test(self,px,py):
            if(px >= self.x and px <= self.breite+self.x and py >=self.y and py <= self.höhe+self.y):
                return True
            else:
                return False
    
    mylist=[]
    
    for i in range(5,310,100):
        for j in range(5,220,75):
            mylist.append(Button(i,j,90,70,ili9342c.CYAN,tft))
    print(mylist)
    
    for button in mylist:
        button.draw()

    player = 1 
   
    while True:
        if(touch.touched>0):
            x = touch.touches[0]['x']
            y = touch.touches[0]['y']

            for button in mylist:
                if button.test(x,y):
                    
                    if button.get_color() == ili9342c.CYAN:
                        
                        if player == 1:
                            button.set_color(ili9342c.RED)
                            player = 2
                        else:
                            button.set_color(ili9342c.BLUE)
                            player = 1
                    
                        button.draw()
        win = checkWin(mylist)
        if(win == False):
            continue
        else:
            print("Blau" if (win == 31) else "Rot", "hat gewonnen") 
            break

def checkWin(myList):

    winConditionsList=[]

    winConditionsList.append([myList[0], myList[1], myList[2]])
    winConditionsList.append([myList[3], myList[4], myList[5]])
    winConditionsList.append([myList[6], myList[7], myList[8]]) 

    winConditionsList.append([myList[0], myList[3], myList[6]])         
    winConditionsList.append([myList[1], myList[4], myList[7]]) 
    winConditionsList.append([myList[2], myList[5], myList[8]])

    winConditionsList.append([myList[6], myList[4], myList[2]]) 
    winConditionsList.append([myList[0], myList[4], myList[8]])

    for winCondition in winConditionsList:
        if(winCondition[0].color != ili9342c.CYAN and winCondition[0].color == winCondition[1].color == winCondition[2].color): 
            return winCondition[0].color
    return False                        
    
if __name__ == "__main__":
    main()
