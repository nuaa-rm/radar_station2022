import { defineConfig } from 'umi';

export default defineConfig({
  nodeModulesTransform: {
    type: 'none',
  },
  routes: [
    { path: '/', component: '@/pages/index' },
  ],
  fastRefresh: {},
  mfsu: {mfName: 'mfMain'},
  dva: {},
  proxy: {
    '/api/ws': {
      target: 'ws://127.0.0.1:43624',
      changeOrigin: true,
      ws: true,
    },
    '/api': {
      target: 'http://127.0.0.1:43624/',
      changeOrigin: true,
    },
    '/socket.io': {
      target: 'http://127.0.0.1:43624/',
      changeOrigin: true,
      onProxyReq: function(request: { setHeader: (arg0: string, arg1: string) => void; }) {
        request.setHeader("origin", "http://127.0.0.1:43624/");
      },
    }
  },
});
