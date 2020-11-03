from flask import Flask,request
import asyncio
app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'hello world'

# example with URL params
@app.route('/post',methods=['POST','GET'])
def show_post():
    # show the post with the given id, the id is an integer
    return 'Post ' + request.args.get('v1') + ' ' + request.args.get('v2')

# example with JSON body
@app.route('/json',methods=['POST'])
def show_json():
    # show the post with the given id, the id is an integer
    v1 = request.get_json()['v1']
    v2 = request.get_json()['v2']
    return {'response':v2}
