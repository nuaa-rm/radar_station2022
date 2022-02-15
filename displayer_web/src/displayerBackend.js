import axois from 'axios';
import {io} from 'socket.io-client';
import {getDvaApp} from "umi";

class DisplayerBackend {
  constructor() {
    const that = this;
    this.timer = setInterval(() => {
      that.init().then();
    }, 100);
  }

  async init() {
    if (getDvaApp()) {
      clearInterval(this.timer);
      this.dispatch = getDvaApp()._store.dispatch;
      const config = await axois.get('/api/getConfig');
      console.log(config)
      this.dispatch({
        type: 'configProvider/init',
        payload: config.data
      })
      this.io = io(config.data.server);
      console.log('backend init')
    }
  }
}

export const displayerBackend = new DisplayerBackend();
