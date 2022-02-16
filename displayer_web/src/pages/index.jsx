import React, {Component} from 'react';
import { connect } from 'dva';

@connect(({ configProvider }) => ({
  configProvider,
}))
class Index extends Component {
  render() {

    return (
      <div>
        <h1>index</h1>

      </div>
    );
  }
}

export default Index;
