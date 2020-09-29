import json
raw = 'px:0.2333;py:0.23444;ox:0.087.'

format = json.loads(raw)
print(format)