import os
import logging, logging.handlers
import sys

import traceback

def handleError(self, record):
    traceback.print_stack()
    traceback.print_exc()
    
logging.Handler.handleError = handleError


def setup_logger(cfg, name=None):
    log_dir = cfg.LOG_DIR
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    debug_h = logging.FileHandler(os.path.join(log_dir, 'debug.log'), mode='w')
    debug_h.setLevel(logging.DEBUG)

    info_h = logging.FileHandler(os.path.join(log_dir, 'info.log'), mode='w')
    info_h.setLevel(logging.INFO)

    warning_h = logging.FileHandler(os.path.join(log_dir, 'error.log'), mode='w')
    warning_h.setLevel(logging.ERROR)

    stream_h = logging.StreamHandler(sys.stdout)
    stream_h.setLevel(1)

    debug_h.setFormatter(  logging.Formatter("%(name)s:%(lineno)s - %(message)s"))
    info_h.setFormatter(   logging.Formatter("%(name)s:%(lineno)s - %(message)s"))
    warning_h.setFormatter(logging.Formatter("%(name)s:%(lineno)s - %(message)s"))
    stream_h.setFormatter( logging.Formatter('%(name)s:%(lineno)s - %(levelname)s - %(message)s'))

    handlers = [
        debug_h, 
        info_h, 
        warning_h, 
        stream_h]

    logger = logging.getLogger()

    logger.setLevel(min([h.level for h in handlers]))

    for h in handlers:
        logger.addHandler(h)

    #return logger

