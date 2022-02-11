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
      target: 'ws://localhost:43624',
      changeOrigin: true,
      ws: true,
    },
    '/api': {
      target: 'http://localhost:43624',
      changeOrigin: true,
    },
  },
});
