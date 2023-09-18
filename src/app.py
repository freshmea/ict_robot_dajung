# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: flask를 사용하는 프로그램
from flask import Flask

app = Flask(__name__)


@app.route("/")
def hello():
    return "<h1>Hello World!</h1>"
