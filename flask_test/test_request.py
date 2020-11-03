import requests

# example with URL params
#for i in range(0,10):
    #response = requests.get(f'http://127.0.0.1:5000/post?v1={i}&v2={2*i}')

# example with JSON body
r = requests.post('http://127.0.0.1:5000/json', json = {'v1':'value','v2':'value2'})
print(r.json())