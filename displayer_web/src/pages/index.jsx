import styles from './index.less';
import React, {Component} from 'react';
import { connect } from 'dva';

@connect(({ rosModel }) => ({
  rosModel,
}))
class Index extends Component {
  render() {
    return (
      <div>
        <img src={ this.props?.rosModel?.cameraOne } alt="cameraOne"/>
      </div>
    );
  }
}

export default Index;
