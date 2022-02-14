import axois from 'axios';
import { io } from 'socket.io-client';

class DisplayerBackend {
  async constructor() {
    const config = await axois.get('/api/getConfig');
    console.log(config)
    this.io = io(config.data.server);
  }
}

export const displayerBackend = new DisplayerBackend();
