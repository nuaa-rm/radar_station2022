#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : webView.py

from flask import Blueprint, redirect

webView = Blueprint('webView', __name__)

@webView.route('/')
def index():
    return redirect('/static/index.html')
