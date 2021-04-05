#Simple test for NeoPixels on Raspberry Pi
#code inspired by https://learn.adafruit.com/neopixels-on-raspberry-pi/python-usage
#image to code software (arduino) https://github.com/TylerTimoJ/LMCSHD
import time
import board
import neopixel
 
 
# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D18
 
# The number of NeoPixels
num_pixels = 128
 
# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB
 
pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER
)
def allOff():
    pixels.fill((0, 0, 0))
def happy():
    #yellow
    r = 255
    g = 255
    b = 0
    pixels[3] = (r , g ,b)
    pixels[4] = (r , g ,b)
    pixels[25] = (r , g ,b)
    pixels[57] = (r , g ,b)
    pixels[62] = (r , g ,b)
    for i in range(4):
        pixels [10 + i] = (r , g ,b)
        pixels[18 + i] = (r , g ,b)
        pixels[27 + i] = (r , g ,b)
    
    for j in range(6):
        pixels [33 + i] = (r , g ,b)
        pixels [41 + i] = (r , g ,b)
        pixels [49 + i] = (r , g ,b)
    #adding 64 for the other matrix
    pixels[3+64] = (r , g ,b)
    pixels[4+64] = (r , g ,b)
    pixels[25+64] = (r , g ,b)
    pixels[57+64] = (r , g ,b)
    pixels[62+64] = (r , g ,b)
    for i in range(4):
        pixels [10+64 + i] = (r , g ,b)
        pixels[18+64 + i] = (r , g ,b)
        pixels[27+64 + i] = (r , g ,b)
    
    for j in range(6):
        pixels [33+64 + i] = (r , g ,b)
        pixels [41+64 + i] = (r , g ,b)
        pixels [49+64 + i] = (r , g ,b)
def neutral():
    #light blue
    r = 204
    g = 255
    b = 255
    pixels[3] = (r , g ,b)
    pixels[4] = (r , g ,b)
    pixels[22] = (r , g ,b)
    pixels[59] = (r , g ,b)
    pixels[60] = (r , g ,b)
    for i in range(4):
        pixels [10 + i] = (r , g ,b)
        pixels[17 + i] = (r , g ,b)
        pixels[50 + i] = (r , g ,b)
    
    for j in range(6):
        pixels [25 + i] = (r , g ,b)
        pixels [33 + i] = (r , g ,b)
        pixels [41 + i] = (r , g ,b)
    #adding 64 for the other matrix
    pixels[3+64] = (r , g ,b)
    pixels[4+64] = (r , g ,b)
    pixels[22+64] = (r , g ,b)
    pixels[59+64] = (r , g ,b)
    pixels[60+64] = (r , g ,b)
    for i in range(4):
        pixels [10 +64 + i] = (r , g ,b)
        pixels[17 + 64 + i] = (r , g ,b)
        pixels[50 + 64 + i] = (r , g ,b)
    
    for j in range(6):
        pixels [25 + 64 + i] = (r , g ,b)
        pixels [33 + 64 + i] = (r , g ,b)
        pixels [41 + 64 + i] = (r , g ,b)
def sad():
    #dark purple 
    r = 90
    g = 37
    b = 101
    for i in range(6):
        pixels [33 + i] = (r , g ,b)
    for j in range(4):
        pixels [42 + i] = (r , g ,b)
    #adding 64 for the other matrix
    for i in range(6):
        pixels [33 + 64 + i] = (r , g ,b)
    for j in range(4):
        pixels [42 + 64 + i] = (r , g ,b)

while(1):
    sad()