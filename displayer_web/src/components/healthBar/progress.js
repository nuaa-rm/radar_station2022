import React, {Component} from 'react';
import { progress, filled } from './progress.css'

class Progress extends Component {
  render() {
    let color = '#888888'
    if (this.props.team === 'red') {
      color = '#f5222d'
    } else if (this.props.team === 'blue') {
      color = '#1890ff'
    }

    let width = 40
    if (this.props.width) {
      width *= this.props.width
    }

    return (
      <div className={progress} style={{width: width.toString() + 'px'}}>
        <div className={filled} style={{width: (this.props.progress * 100).toString() + '%', backgroundColor: color}} />
      </div>
    );
  }
}

export default Progress;
