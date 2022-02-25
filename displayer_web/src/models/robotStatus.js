export default {
  namespace: 'robotStatus',
  state: {red: {}, blue: {}},
  reducers: {
    'refresh' (state, { payload }) {
      return {...payload}
    },
  },
};
