#!/usr/bin/env python3

from pygpmp2 import PlanarGPMP2, PlanarGPMP2Settings
import flask
from flask import Flask, render_template, url_for, request
import pickle
import base64
import numpy as np
import cv2
import json
# import tensorflow as tf

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


# Skip the unused prefix of the base64 encoded image.
init_Base64 = 21

# Initializing new Flask instance. Find the html template in "templates".
app = flask.Flask(__name__, template_folder='templates')

#First route : Render the initial drawing template


## GPMP2, default params
cell_size = 0.1
opts = PlanarGPMP2Settings()
opts.use_vehicle_dynamics = True
opts.total_time_sec = 1.0
opts.total_time_step = 16
opts.cost_sigma = 0.01
opts.radius = 1.0  # 0.0002
opts.verbosity = 'SILENT'
gpmp2 = PlanarGPMP2(opts)


@app.route('/')
def home():
    args = dict(
        cell_size=cell_size,
        total_time_sec=opts.total_time_sec,
        total_time_step=opts.total_time_step,
        check_inter=opts.check_inter,
        cost_sigma=opts.cost_sigma,
        radius=opts.radius
    )
    return render_template('main.html', opts=args)


@app.route('/plan')
def plan():
    """ xhr request for invoking planning """
    if not request.is_xhr:
        return ' '
    img = request.args.get('img')

    # Conditionally update GPMP2 Options.
    update_opts = False
    for key in request.args:
        if not hasattr(opts, key):
            continue

        # NOTE(ycho): HACK to enable
        # casting to same target type.
        cls = type(getattr(opts, key))
        setattr(opts, key, cls(request.args[key]))

        update_opts = True

    if update_opts:
        gpmp2.SetOpts(opts)

    # Decode Image.
    img = img[init_Base64:]
    img = base64.b64decode(img)
    img = np.asarray(bytearray(img), dtype=np.uint8)
    img = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)

    # NOTE(ycho): Additional param ...
    init = (5, 5)
    goal = (img.shape[0] - 5, img.shape[1] - 5)
    if 'initX' in request.args.keys():
        init = (float(request.args['initY']), float(request.args['initX']))
    if 'goalX' in request.args.keys():
        goal = (float(request.args['goalY']), float(request.args['goalX']))

    if 'cell_size' in request.args.keys():
        cell_size = float(request.args['cell_size'])

    gpmp2.Init(img, cell_size)
    try:
        out = gpmp2.Plan(init, goal)
    except RuntimeError as e:
        return('ERROR : {}'.format(e))
    return json.dumps(out)


if __name__ == '__main__':
	app.run(host='0.0.0.0', port=5000, debug=False)
