export default {
  namespace: 'minimap',
  state: {test1: {color: 'red', text: 'test', data: [[0.2, 0.5], [0.6, 0.7], [0.2, 0.7]]}, test2: {color: 'green', data: [[0.3, 0.5]], text: '1'}},
  reducers: {
    'refresh' (state, { payload }) {
      return {...payload}
    },
  },
};
