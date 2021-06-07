#!/usr/bin/env python

from flask import Flask, request, render_template, redirect, url_for, abort
# from flask_cors import CORS


app = Flask(__name__)
# CORS(app)

@app.route('/')
def main_web():
  return 'main web'

@app.route('/button')
def button():
  return render_template('button.html')

@app.route('/joystick')
def joystick():
  return render_template('joystick.html')

@app.route('/testDraw')
def testDraw():
  return render_template('testDraw.html')

@app.route('/testNavigation')
def testNavigation():
  return render_template('testNavigation.html')

@app.route('/mapping')
def mapping():
  return render_template('mapping.html')

@app.route('/navigation')
def navigation():
  return render_template('navigation.html')


if __name__ == '__main__':	
   app.run(host='0.0.0.0', port=5000, debug=True)