from PIL import Image, ImageDraw, ImageFont
import sys

### Create 1024,1024 pixel image with a white background.
img = Image.new("RGB", (1024, 1024), color = (255,255,255))

### Take text to be drawn on the image from the command terminal.
text = sys.argv[1]

### Chose favourite font and set size of the font.
fnt = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 150, encoding="unic")
d = ImageDraw.Draw(img)

d.text(xy=(320,420), text = text , font = fnt, fill=(0,0,0))

### Save image as .png file.
img.save(text+'.png')
