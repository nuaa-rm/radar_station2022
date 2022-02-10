import styles from './index.less';
import React, {Component} from 'react';
import { connect } from 'dva';
import Calibrator from "../components/calibrator";

@connect(({ rosModel }) => ({
  rosModel,
}))
class Index extends Component {
  render() {
    return (
      <div>
        <h1>test</h1>
        <Calibrator />
      </div>
    );
  }
}

export default Index;
